import os
assert os.environ.get('ROS_DOMAIN_ID')=='84'
from ur7e_auto_esdf.mapper import main
main()
