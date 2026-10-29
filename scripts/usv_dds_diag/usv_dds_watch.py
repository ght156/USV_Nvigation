#!/usr/bin/env python3
"""USV DDS / scheduler flight recorder. Python standard library only, read-only."""
import argparse, collections, datetime, json, os, pathlib, re, shutil, signal, subprocess, sys, time

DEFAULT_NAMES = 'controller_server,bt_navigator,nav_status,mission_bridge,mqtt_bridgev2,video_camera,apriltag,decision,livox,mavros'
FAULT = re.compile(r'CTRL_STUCK|Control loop missed|RcvbufErrors|BOAT_CONTROLLER\s*\(?3006|BATTERY\s*\(?5001|7001 ACTIVE|Preempt active mission|cancel requested', re.I)
stop = False
manual = False

def sig_stop(*_):
    global stop
    stop = True

def sig_event(*_):
    global manual
    manual = True

def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='milliseconds')

def read(path, max_bytes=200000):
    try:
        with open(path, 'rb') as f:
            return f.read(max_bytes).decode('utf-8', 'replace')
    except (OSError, PermissionError):
        return ''

def numbers(s):
    return [int(v) for v in re.findall(r'\d+', s)]

def proc_stat(pid, tid=None):
    root = f'/proc/{pid}/task/{tid}' if tid is not None else f'/proc/{pid}'
    s = read(root + '/stat', 8192)
    if not s or ') ' not in s:
        return None
    vals = s.rsplit(') ', 1)[1].split()
    try:
        # vals[0] is field 3: state
        return {'state':vals[0], 'utime':int(vals[11]), 'stime':int(vals[12]),
                'threads':int(vals[17]), 'rss_pages':int(vals[21]), 'processor':int(vals[36])}
    except (IndexError, ValueError):
        return None

def schedstat(path):
    v = numbers(read(path, 200))
    return v[:3] if len(v) >= 3 else None

def status_small(path):
    data = {}
    for line in read(path, 16000).splitlines():
        if line.startswith(('Name:', 'State:', 'VmRSS:', 'VmHWM:', 'voluntary_ctxt_switches:', 'nonvoluntary_ctxt_switches:')):
            k, _, v = line.partition(':')
            data[k] = v.strip()
    return data

def cpu_snapshot():
    d = {}
    for line in read('/proc/stat').splitlines():
        p = line.split()
        if p and (p[0] == 'cpu' or re.fullmatch(r'cpu\d+', p[0])):
            try: d[p[0]] = list(map(int, p[1:]))
            except ValueError: pass
    return d

def net_snapshot():
    d = {}
    for path in ('/proc/net/snmp','/proc/net/netstat'):
        lines = read(path).splitlines()
        for a,b in zip(lines[::2], lines[1::2]):
            h, v = a.split(), b.split()
            if h and v and h[0] == v[0] and len(h) == len(v):
                for k, n in zip(h[1:], v[1:]):
                    try: d[h[0][:-1] + '.' + k] = int(n)
                    except ValueError: pass
    return d

def socket_snapshot():
    out = []
    for p in ('/proc/net/udp','/proc/net/udp6'):
        for l in read(p).splitlines()[1:]:
            x = l.split()
            if len(x) >= 13:
                try:
                    out.append({'proto':p.rsplit('/',1)[-1], 'local':x[1], 'remote':x[2],
                                'rx_queue_hex':x[4].split(':')[1], 'drops':int(x[12]), 'inode':x[9]})
                except (IndexError, ValueError): pass
    return sorted(out, key=lambda z:z['drops'], reverse=True)[:25]

def shm_snapshot():
    out = {'shm_files':0,'shm_bytes':0,'fastdds_files':0,'fastdds_bytes':0,'largest':[]}
    for parent in ('/dev/shm',):
        try: entries = list(os.scandir(parent))
        except OSError: continue
        sizes = []
        for e in entries:
            try: s = e.stat(follow_symlinks=False).st_size
            except OSError: continue
            out['shm_files'] += 1; out['shm_bytes'] += s
            if re.search(r'fast|fastrtps|dds|sem\.', e.name, re.I):
                out['fastdds_files'] += 1; out['fastdds_bytes'] += s
            sizes.append([e.name[:110], s])
        out['largest'] = sorted(sizes, key=lambda v:v[1], reverse=True)[:12]
    return out

def select_processes(patterns):
    res = {}
    me = os.getpid()
    try: dirs = os.listdir('/proc')
    except OSError: return res
    for name in dirs:
        if not name.isdigit(): continue
        pid = int(name)
        if pid == me: continue
        cmd = read(f'/proc/{pid}/cmdline', 20000).replace('\0', ' ').strip()
        comm = read(f'/proc/{pid}/comm', 200).strip()
        if any(p in (cmd + ' ' + comm).lower() for p in patterns):
            res[pid] = {'comm':comm, 'cmdline':cmd[:400]}
    return res

def thread_details(pid, previous, now, max_threads):
    records = []
    try: tids = [int(x) for x in os.listdir(f'/proc/{pid}/task') if x.isdigit()]
    except OSError: return []
    for tid in tids:
        root = f'/proc/{pid}/task/{tid}'
        sch = schedstat(root + '/schedstat')
        st = proc_stat(pid, tid)
        if not sch or not st: continue
        key = f'{pid}:{tid}'
        old = previous.get(key)
        now[key] = sch
        delayed = (sch[1] - old[1]) if old else None
        runtime = (sch[0] - old[0]) if old else None
        records.append({'tid':tid,'name':read(root+'/comm',100).strip(),'state':st['state'],
                        'cpu':st['processor'], 'runtime_ns_delta':runtime,
                        'runqueue_wait_ns_delta':delayed, 'sched_count_delta':sch[2]-old[2] if old else None})
    # Preserve threads with largest wait and CPU time, plus the executor/main threads.
    records.sort(key=lambda a:max(a['runqueue_wait_ns_delta'] or 0, a['runtime_ns_delta'] or 0), reverse=True)
    selected = records[:max_threads]
    if not any(a['tid'] == pid for a in selected):
        main = next((a for a in records if a['tid']==pid),None)
        if main: selected.append(main)
    return {'total_threads':len(records),'top_threads':selected,
            'total_runqueue_wait_ns_delta':sum(a['runqueue_wait_ns_delta'] or 0 for a in records),
            'max_thread_wait_ns_delta':max((a['runqueue_wait_ns_delta'] or 0 for a in records),default=0)}

def system_snapshot():
    return {'loadavg':read('/proc/loadavg').strip(), 'meminfo':{k:v.strip() for k,v in (x.split(':',1) for x in read('/proc/meminfo').splitlines() if ':' in x) if k in ('MemAvailable','MemFree','SwapFree','Dirty','Writeback')},
            'pressure':{x:read('/proc/pressure/'+x).strip() for x in ('cpu','memory','io')},
            'softnet_stat':read('/proc/net/softnet_stat',4096).splitlines()[:24],
            'netdev':read('/proc/net/dev').strip(), 'interrupts':read('/proc/interrupts',3000)}

def copy_log_tails(outdir, rosroot, max_mb):
    dest = outdir / 'event_logs'; dest.mkdir(exist_ok=True)
    try: paths = [p for p in rosroot.rglob('*.log') if p.is_file()]
    except OSError: return
    paths.sort(key=lambda x:x.stat().st_mtime if x.exists() else 0, reverse=True)
    cap = int(max_mb*1024*1024); total = 0
    for p in paths[:80]:
        if total >= cap: break
        try:
            size = p.stat().st_size
            n = min(size, 300000, cap-total)
            with p.open('rb') as f:
                f.seek(max(0,size-n)); data=f.read(n)
            safe = re.sub(r'[^a-zA-Z0-9_.-]', '_', str(p.relative_to(rosroot)))
            (dest/safe).write_bytes(data)
            total += len(data)
        except (OSError,ValueError): pass

def watch_logs(root, offsets, dst, emit):
    try:
        paths = [p for p in root.rglob('*.log') if p.is_file()]
    except OSError: return
    for p in paths[:250]:
        key = str(p)
        try:
            size = p.stat().st_size
            if key not in offsets:
                offsets[key] = max(0, size - 30000)
            if size < offsets[key]: offsets[key] = 0
            with p.open('rb') as f:
                f.seek(offsets[key]); raw = f.read(150000)
                offsets[key] = f.tell()
            for line in raw.decode('utf-8','replace').splitlines():
                if FAULT.search(line):
                    rec={'timestamp':utc(),'file':key,'line':line[:1200]}
                    dst.write(json.dumps(rec,ensure_ascii=False)+'\n')
                    emit(line)
        except OSError: pass

def main():
    global manual
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',required=True,choices=['raw','compressed'],help='Label only; does not change MQTT configuration')
    ap.add_argument('--out',default='./usv_diag_runs')
    ap.add_argument('--interval',type=float,default=1.0)
    ap.add_argument('--minutes',type=float,default=180)
    ap.add_argument('--names',default=DEFAULT_NAMES)
    ap.add_argument('--threads',type=int,default=16)
    ap.add_argument('--ros-log-dir',default=str(pathlib.Path.home()/'.ros/log'))
    ap.add_argument('--log-scan-interval',type=float,default=3)
    ap.add_argument('--max-event-log-mb',type=float,default=15)
    ap.add_argument('--topic',action='append',default=[],help='Optional `ros2 topic echo` watch (e.g. /nav_status); message type auto-detected')
    args = ap.parse_args()
    if args.interval < 0.2 or args.minutes <= 0: ap.error('interval must >=0.2 and minutes >0')
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = pathlib.Path(args.out).expanduser().resolve() / f'{stamp}_{args.mode}'
    out.mkdir(parents=True)
    meta = {'started':utc(),'mode':args.mode,'argv':sys.argv,'host':os.uname().nodename,
            'kernel':os.uname().release,'clock_ticks':os.sysconf('SC_CLK_TCK'),
            'page_size':os.sysconf('SC_PAGE_SIZE'), 'boot_id':read('/proc/sys/kernel/random/boot_id').strip(),
            'notes':'Counters cumulative since boot; use differences. Per-thread runqueue wait uses /proc/*/schedstat. No DDS topic attribution from UDP counters.'}
    (out/'meta.json').write_text(json.dumps(meta,indent=2,ensure_ascii=False))
    names = [x.strip().lower() for x in args.names.split(',') if x.strip()]
    old_sched = {}; log_offsets = {}; recent = collections.deque(maxlen=200)
    rosroot = pathlib.Path(args.ros_log_dir).expanduser()
    ros_procs = []
    def record_event(msg):
        nonlocal last_event
        if 'CTRL_STUCK' in msg.upper() or '7001 ACTIVE' in msg.upper() or msg == 'MANUAL_TRIGGER':
            if time.monotonic() - last_event >= 20:
                last_event = time.monotonic()
                event_path = out/f'event_{datetime.datetime.now().strftime("%H%M%S")}.json'
                event_path.write_text(json.dumps({'at':utc(),'reason':msg[:500], 'recent_samples':list(recent)},ensure_ascii=False))
                copy_log_tails(out,rosroot,args.max_event_log_mb)
                print('EVENT snapshot:',event_path,flush=True)
    last_event = -1e20
    signal.signal(signal.SIGINT,sig_stop); signal.signal(signal.SIGTERM,sig_stop)
    if hasattr(signal,'SIGUSR1'): signal.signal(signal.SIGUSR1,sig_event)
    with (out/'samples.jsonl').open('a',buffering=1) as samples, (out/'fault_lines.jsonl').open('a',buffering=1) as events, (out/'topic_echo.txt').open('a',buffering=1) as topicfile:
        for topic in args.topic:
            if shutil.which('ros2'):
                try:
                    p = subprocess.Popen(['ros2','topic','echo',topic],stdout=topicfile,stderr=topicfile,
                        env={**os.environ,'PYTHONUNBUFFERED':'1'}, start_new_session=True)
                    ros_procs.append(p)
                except OSError: pass
        start = time.monotonic(); nxt = start; last_scan = -1e20; iteration=0
        print(f'Writing {out} | mode={args.mode} | PID={os.getpid()} | stop: Ctrl-C | event: kill -USR1 {os.getpid()}',flush=True)
        try:
            while not stop and time.monotonic()-start < args.minutes*60:
                t0 = time.monotonic(); now_sched = {}; processes = {}
                for pid, identity in select_processes(names).items():
                    st = proc_stat(pid)
                    if not st: continue
                    processes[str(pid)] = {**identity,'stat':st,'status':status_small(f'/proc/{pid}/status'),
                                            'schedstat':schedstat(f'/proc/{pid}/schedstat'),
                                            'threads':thread_details(pid,old_sched,now_sched,args.threads)}
                old_sched = now_sched
                row = {'at':utc(),'monotonic_s':time.monotonic(),'iteration':iteration,'mode':args.mode,
                       'processes':processes,'cpu':cpu_snapshot(),'net':net_snapshot(),
                       'udp_sockets':socket_snapshot(),'shm':shm_snapshot(),'system':system_snapshot()}
                samples.write(json.dumps(row,ensure_ascii=False,separators=(',',':'))+'\n')
                recent.append({'at':row['at'],'net':{k:v for k,v in row['net'].items() if 'RcvbufErrors' in k or 'InErrors' in k},
                               'thread_wait_ms':{p:round(x['threads']['max_thread_wait_ns_delta']/1e6,2) for p,x in processes.items()},
                               'proc_names':{p:x['comm'] for p,x in processes.items()}})
                if time.monotonic()-last_scan >= args.log_scan_interval:
                    watch_logs(rosroot,log_offsets,events,record_event)
                    last_scan = time.monotonic()
                if manual:
                    manual = False
                    events.write(json.dumps({'timestamp':utc(),'line':'MANUAL_TRIGGER'})+'\n')
                    record_event('MANUAL_TRIGGER')
                trigger_file = out/'TRIGGER'
                if trigger_file.exists():
                    try: trigger_file.unlink()
                    except OSError: pass
                    record_event('MANUAL_TRIGGER')
                if iteration % 30 == 0:
                    print(row['at'], 'tracked_pids=',len(processes),'samples=',iteration+1,flush=True)
                iteration+=1; nxt+=args.interval
                remaining=nxt-time.monotonic()
                if remaining>0: time.sleep(remaining)
                elif remaining < -args.interval: nxt=time.monotonic()
        finally:
            for p in ros_procs:
                if p.poll() is None:
                    p.terminate()
                    try: p.wait(timeout=2)
                    except subprocess.TimeoutExpired: p.kill()
            (out/'finished.txt').write_text('finished_at='+utc()+'\n')
            print('Finished:',out,flush=True)

if __name__=='__main__':
    main()
