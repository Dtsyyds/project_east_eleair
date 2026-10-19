from pathlib import Path
import ast,types,json,sys
ROOT=Path(__file__).resolve().parents[2];REPORT=ROOT/'tank_inspection_system/reports/refactor_v1'
sys.path.insert(0,str(ROOT/'physic_bev/scripts'))
from sensor_msgs.msg import Image
from rclpy.serialization import serialize_message
import rosbag2_py
import validate_vehicle_collection as new
oldpath=REPORT/'backups/physic_bev/scripts/validate_vehicle_collection.py'
old=types.ModuleType('baseline_validator');old.__file__=str(oldpath)
exec(compile(oldpath.read_text(),str(oldpath),'exec'),old.__dict__)
class Reader:
 def __init__(self):self.remaining=3
 def open(self,*args):pass
 def get_all_topics_and_types(self):return [types.SimpleNamespace(name='/camera/front/image_raw',type='sensor_msgs/msg/Image')]
 def has_next(self):return self.remaining>0
 def read_next(self):
  self.remaining-=1
  return '/camera/front/image_raw',serialize_message(Image()),1000000000+(2-self.remaining)*100000000
original=rosbag2_py.SequentialReader
rosbag2_py.SequentialReader=Reader
try:
 a=old.inspect_bag_messages(Path('fixture-reader-no-mcap'))
 b=new.inspect_bag_messages(Path('fixture-reader-no-mcap'))
finally:rosbag2_py.SequentialReader=original
stats=a['topics']['/camera/front/image_raw']
old_monotonic=stats.get('bag_timestamp_regressions',0)==0 and stats.get('header_stamp_regressions',0)==0
new_valid=new.timestamp_contract_valid('/camera/front/image_raw',b['topics']['/camera/front/image_raw'])
def eddy_expr(text):
 tree=ast.parse(text)
 for node in ast.walk(tree):
  if isinstance(node,ast.Dict):
   for k,v in zip(node.keys,node.values):
    if isinstance(k,ast.Constant) and k.value=='eddy_driver_status_error_zero':return ast.Expression(v)
old_eddy=eval(compile(eddy_expr(oldpath.read_text()),str(oldpath),'eval'),dict(requires_eddy_status=False,latest_eddy={}))
new_eddy=eval(compile(eddy_expr((ROOT/'physic_bev/scripts/validate_vehicle_collection.py').read_text()),'current','eval'),dict(requires_eddy_status=False,latest_eddy={},enabled={'eddy':True},eddy_error_seen=True))
result={'fixture':'real ROS Image CDR through injected reader; not MCAP plugin acceptance','zero_header_total':3,'baseline_stats':stats,'modified_stats':b['topics']['/camera/front/image_raw'],'baseline_false_timestamp_acceptance':old_monotonic,'modified_timestamp_rejected':not new_valid,'baseline_monitor_eddy_error_false_acceptance':old_eddy,'modified_monitor_eddy_error_rejected':not new_eddy,'hardware_executed':False}
assert all([old_monotonic,not new_valid,old_eddy,not new_eddy])
(REPORT/'p1_timestamp_eddy_reproduction.json').write_text(json.dumps(result,indent=2)+'\n')
print('P1 zero timestamp and persisted Eddy error false acceptance reproduced; fixes reject both')
