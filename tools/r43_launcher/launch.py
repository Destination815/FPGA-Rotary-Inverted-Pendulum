"""Run the unchanged R43 compiler and preserve startup errors beside the project."""
from pathlib import Path
import contextlib
import datetime
import runpy
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]


class Tee:
    def __init__(self, console, log):
        self.console = console
        self.log = log

    def write(self, text):
        self.console.write(text)
        self.log.write(text)
        self.log.flush()
        return len(text)

    def flush(self):
        self.console.flush()
        self.log.flush()

    def isatty(self):
        return self.console.isatty()

    @property
    def encoding(self):
        return getattr(self.console, 'encoding', 'utf-8')


def main(args=None):
    args = list(sys.argv[1:] if args is None else args)
    compiler = ROOT / 'tools' / 'compile_r40_once.py'
    try:
        with (ROOT / 'R43_startup.log').open('a', encoding='utf-8') as log:
            with contextlib.redirect_stdout(Tee(sys.stdout, log)), contextlib.redirect_stderr(Tee(sys.stderr, log)):
                print('Started:', datetime.datetime.now().isoformat(timespec='seconds'), flush=True)
                print('Python:', sys.executable, flush=True)
                print('Project:', ROOT, flush=True)
                old_argv = sys.argv
                try:
                    if args == ['--check']:
                        namespace = runpy.run_path(str(compiler), run_name='r42_startup_check')
                        snapshot = namespace['inputs'](ROOT)
                        print('STARTUP_CHECK_PASS: build', snapshot['build'],
                              '; FPGA inputs', len(snapshot['source_sha256']),
                              '; no Gowin process started.', flush=True)
                        return 0
                    sys.argv = [str(compiler)] + args
                    runpy.run_path(str(compiler), run_name='__main__')
                    return 0
                except SystemExit as exc:
                    if exc.code is None:
                        return 0
                    if isinstance(exc.code, int):
                        return exc.code
                    print(exc.code, file=sys.stderr)
                    return 1
                except KeyboardInterrupt:
                    print('Compiler launch interrupted.', file=sys.stderr)
                    return 130
                except Exception:
                    traceback.print_exc()
                    return 2
                finally:
                    sys.argv = old_argv
    except OSError as exc:
        print('Could not write startup log:', exc, file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
