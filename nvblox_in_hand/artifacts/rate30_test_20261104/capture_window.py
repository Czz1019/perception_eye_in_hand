import sys, subprocess, re, ctypes, time
from PyQt5.QtWidgets import QApplication
app=QApplication([])
tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
windows=re.findall(r'(0x[0-9a-f]+) "([^"\n]*RViz[^"\n]*)"',tree)
if not windows:raise RuntimeError('No RViz window found')
wid=int(windows[0][0],16)
x=ctypes.cdll.LoadLibrary('libX11.so.6')
x.XOpenDisplay.restype=ctypes.c_void_p
x.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
x.XFlush.argtypes=[ctypes.c_void_p]
x.XCloseDisplay.argtypes=[ctypes.c_void_p]
d=x.XOpenDisplay(None)
if d:x.XRaiseWindow(d,wid);x.XFlush(d)
time.sleep(.6);app.processEvents()
pix=app.primaryScreen().grabWindow(wid)
if pix.isNull() or not pix.save(sys.argv[1]):raise RuntimeError('Screenshot failed')
if d:x.XCloseDisplay(d)
print(sys.argv[1],pix.width(),pix.height())
