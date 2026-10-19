"""Run isolated live Gazebo acceptance steps; never starts a real robot."""
import argparse,json,os,subprocess,time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--scan-timeout',type=float,default=1200.);a=p.parse_args();out=Path(a.output).resolve();ws=Path(__file__).resolve().parents[3]
results=[]
def status():
 try:return json.loads((out/'status.json').read_text())
 except (ValueError,FileNotFoundError):return {}
def wait(predicate,seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  s=status()
  if predicate(s):return s
  if s.get('stopped'):raise RuntimeError(str(s))
  time.sleep(.25)
 raise TimeoutError(str(status()))
def control(*args):
 subprocess.run([str(ws/'ur7e_gazebo_esdf_sage_continuous_control.sh'),*map(str,args)],check=True,timeout=20,stdout=subprocess.DEVNULL)
def target(xyz):
 control('target',*xyz)
 # The control client waits after publish, so a very short rejected search may
 # already be finished. The event revision disambiguates prior goal outcomes.
 events=[json.loads(l) for l in (out/'events.jsonl').read_text().splitlines()]
 rev=[e['revision'] for e in events if e['kind']=='target'][-1]
 return rev
def outcome(rev,seconds=190):
 def finished(s):
  events=[json.loads(l) for l in (out/'events.jsonl').read_text().splitlines()]
  return any(e['kind'] in ['goal_reached','goal_rejected'] and e['revision']==rev for e in events)
 wait(finished,seconds)
 return [e for e in map(json.loads,(out/'events.jsonl').read_text().splitlines()) if e.get('revision')==rev and e['kind'] in ['goal_reached','goal_rejected']][-1]
def record(name,**kwargs):
 results.append(dict(test=name,**kwargs));(out/'system_scenario.json').write_text(json.dumps(results,indent=2));print(results[-1],flush=True)
def probe(name):
 env=dict(os.environ,ROS_DOMAIN_ID='87')
 subprocess.run(['/home/wqj/miniconda3/envs/whole_control/bin/python','-B',str(ws/'src/ur7e_gazebo_esdf_sage_continuous/test/probe_runtime.py'),str(out),'--expect-hold'],env=env,check=True,timeout=30,stdout=subprocess.DEVNULL)
 (out/(name+'.json')).write_text((out/'runtime_probe.json').read_text())
try:
 wait(lambda s:s.get('phase') in ['GOAL_REACHED','GOAL_REJECTED'],a.scan_timeout)
 record('fresh_scan_default',passed=status()['phase']=='GOAL_REACHED',status=status())
 rev=target([.6027275713,-.0729611598,.4420525377]);event=outcome(rev);record('low_between_walls',passed=event['kind']=='goal_reached',event=event)
 if status().get('planning_policy')=='bounded_unknown':
  rev=target([.50,.20,.23]);event=outcome(rev,45);probe('occupied_goal_hold');record('occupied_goal_hold',passed=event['kind']=='goal_rejected',event=event)
 # Return toward the default target, then retarget while it is executing.
 rev=target([.50,0,.60]);wait(lambda s:s.get('phase')=='REACHING',40);time.sleep(1.)
 rev=target([.30,.10,.94]);event=outcome(rev);record('retarget_while_moving',passed=event['kind']=='goal_reached',event=event)
 uid=status()['map_uid'];frames=status()['frames']
 rev=target([2.,2.,2.]);event=outcome(rev,45);probe('unknown_goal_hold');record('unknown_goal_hold',passed=event['kind']=='goal_rejected',event=event)
 rev=target([.50,0,.60]);wait(lambda s:s.get('phase')=='REACHING',40);time.sleep(1.)
 control('test_feedback_loss');wait(lambda s:s.get('stopped'),10);probe('feedback_loss_hold')
 record('feedback_loss_hold',passed=status()['stopped'] and status()['map_uid']==uid and status()['frames']==frames,status=status())
except Exception as exc:
 record('scenario_exception',passed=False,error=str(exc));raise

if not all(r.get("passed",False) for r in results):raise SystemExit(1)
