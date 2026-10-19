"""Launch the actual entrypoint without devices and check clean SIGINT exit."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
from fixtures import config


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    config(output)
    workspace=Path(__file__).resolve().parents[3]
    logfile=output/'launch.log'
    with logfile.open('w') as stream:
        process=subprocess.Popen([str(workspace/'run_ur7e_real_static.sh'),
                                  'config:='+str(output/'config.yaml')],
                                 cwd=workspace,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+20
            while 'Ready for stationary-view scanning' not in logfile.read_text():
                assert process.poll() is None,logfile.read_text()
                if time.monotonic()>deadline:raise TimeoutError(logfile.read_text())
                time.sleep(.1)
            assert 'execution_enabled=False' in logfile.read_text()
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=15)==0,logfile.read_text()
            content=logfile.read_text()
            assert 'process has finished cleanly' in content and 'Traceback' not in content,content
        finally:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGKILL);process.wait()
    print(logfile.read_text())
    print('PASS: actual root launcher ready in preview mode and exited cleanly without devices')


if __name__=='__main__':main()
