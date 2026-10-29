#!/usr/bin/env python3
"""Summarize a usv_dds_watch run without third-party dependencies."""
import collections, json, pathlib, statistics, sys

def percentile(v, q):
    if not v:return None
    v=sorted(v); return round(v[int((len(v)-1)*q)],2)

def main():
    if len(sys.argv)!=2:
        print('Usage: python3 analyze_usv_watch.py RUN_DIRECTORY');sys.exit(2)
    path=pathlib.Path(sys.argv[1]); rows=[]
    with (path/'samples.jsonl').open() as f:
        for line in f:
            try: rows.append(json.loads(line))
            except json.JSONDecodeError: pass
    if not rows: print('No samples');return
    print('Mode:',rows[0]['mode'],'Start:',rows[0]['at'],'End:',rows[-1]['at'],'Samples:',len(rows))
    print('Time span(s):',round(rows[-1]['monotonic_s']-rows[0]['monotonic_s'],1))
    for key in ('Udp.RcvbufErrors','Udp.InErrors','Udp.InDatagrams','Udp.OutDatagrams','TcpExt.TCPBacklogDrop'):
        a=rows[0]['net'].get(key); b=rows[-1]['net'].get(key)
        if a is not None and b is not None:
            print(key, 'delta=',b-a,'per_second=',round((b-a)/max(.01,rows[-1]['monotonic_s']-rows[0]['monotonic_s']),2))
    by_name=collections.defaultdict(list); wait=collections.defaultdict(list); cpu=collections.defaultdict(list)
    for row in rows:
        for p,v in row['processes'].items():
            name=v['comm']; by_name[name].append((row['at'],v['threads']['max_thread_wait_ns_delta']/1e6,p))
            wait[name].append(v['threads']['max_thread_wait_ns_delta']/1e6)
            cpu[name].append((row['monotonic_s'],v['stat']['utime']+v['stat']['stime']))
    hz=json.loads((path/'meta.json').read_text())['clock_ticks']
    print('\nProcess max thread runqueue wait per sample (ms; not message latency):')
    for name, values in sorted(wait.items(),key=lambda z:max(z[1]),reverse=True):
        peak=max(by_name[name],key=lambda x:x[1]); seq=cpu[name]
        pct=round(100*((seq[-1][1]-seq[0][1])/hz)/(seq[-1][0]-seq[0][0]),1) if len(seq)>1 and seq[-1][0]>seq[0][0] else None
        print(f'  {name:26} p50={percentile(values,.50)} p95={percentile(values,.95)} p99={percentile(values,.99)} max={round(peak[1],2)} at={peak[0]} pid={peak[2]} cpu_core_pct~={pct}')
    events=path/'fault_lines.jsonl'
    if events.exists():
        lines=events.read_text(errors='replace').splitlines()
        print('\nMatched log lines:',len(lines))
        for ln in lines[-12:]:
            try:
                e=json.loads(ln); print(' ',e.get('timestamp'),e.get('line','')[:150])
            except json.JSONDecodeError: pass
    print('\nNOTE: CPU % is one-core equivalent and assumes PID lifetime spans sample window. Per-PID name aggregation may include restarts.')
if __name__=='__main__':main()
