"""Use the production mapper, asserting that no ESDF API can be called."""
import os
assert os.environ.get('ROS_DOMAIN_ID')=='84'
from realtime_nvblox.nvblox_backend import NvbloxBackend
from ur7e_auto_reconstruction.mapper import main

def forbidden(*args,**kwargs):
    raise AssertionError('This reconstruction must never compute an ESDF')

NvbloxBackend.update_esdf=forbidden
main()
