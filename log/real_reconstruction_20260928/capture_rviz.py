import ctypes,time,re,subprocess
from pathlib import Path
from PIL import ImageGrab
x=ctypes.CDLL('libX11.so.6')
x.XOpenDisplay.restype=ctypes.c_void_p
x.XDefaultRootWindow.argtypes=[ctypes.c_void_p];x.XDefaultRootWindow.restype=ctypes.c_ulong
x.XInternAtom.argtypes=[ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int];x.XInternAtom.restype=ctypes.c_ulong
class Data(ctypes.Union):_fields_=[('b',ctypes.c_char*20),('s',ctypes.c_short*10),('l',ctypes.c_long*5)]
class Message(ctypes.Structure):_fields_=[('type',ctypes.c_int),('serial',ctypes.c_ulong),('send_event',ctypes.c_int),('display',ctypes.c_void_p),('window',ctypes.c_ulong),('message_type',ctypes.c_ulong),('format',ctypes.c_int),('data',Data)]
class Event(ctypes.Union):_fields_=[('xclient',Message),('pad',ctypes.c_long*24)]
x.XSendEvent.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_long,ctypes.POINTER(Event)]
x.XFlush.argtypes=[ctypes.c_void_p]
d=x.XOpenDisplay(None);root=x.XDefaultRootWindow(d);e=Event();e.xclient.type=33;e.xclient.display=d;e.xclient.window=0x5000106;e.xclient.message_type=x.XInternAtom(d,b'_NET_ACTIVE_WINDOW',0);e.xclient.format=32;e.xclient.data.l[0]=2
x.XSendEvent(d,root,0,(1<<20)|(1<<19),ctypes.byref(e));x.XFlush(d);time.sleep(.6)
s=subprocess.check_output(['xwininfo','-id','0x5000106'],text=True)
def val(name):return int(re.search(name+r':\s*(-?\d+)',s).group(1))
a,b,w,h=val('Absolute upper-left X'),val('Absolute upper-left Y'),val('Width'),val('Height')
print(a,b,w,h)
ImageGrab.grab(bbox=(a,b,a+w,b+h),xdisplay=':1').save('log/real_reconstruction_20260928/real/scan_20260928_130419_942f86/final_rviz.png')
