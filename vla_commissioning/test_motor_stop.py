import unittest
from types import SimpleNamespace
from motor_stop import bench_disarm

class StopTests(unittest.TestCase):
 def test_normal_disarm_no_force(self):
  calls=[]
  r=bench_disarm(lambda **kw:(calls.append(kw) or SimpleNamespace(success=True,result=0)),lambda **kw:self.fail('unexpected force'),lambda:False,sleep=lambda _:None)
  self.assertTrue(r['disarmed_confirmed']);self.assertEqual(calls,[{'value':False}])
 def test_rejected_disarm_force_is_disarm_only(self):
  state=[True];calls=[]
  def force(**kw):
   calls.append(kw);state[0]=False;return SimpleNamespace(success=True,result=0)
  r=bench_disarm(lambda **kw:SimpleNamespace(success=False,result=1),force,lambda:state[0],propellers_removed_bench=True,sleep=lambda _:None)
  self.assertTrue(r['disarmed_confirmed']);self.assertEqual(calls[0]['command'],400);self.assertEqual(calls[0]['param1'],0);self.assertEqual(calls[0]['param2'],21196)
 def test_force_disabled_outside_confirmed_bench(self):
  r=bench_disarm(lambda **kw:SimpleNamespace(success=False,result=1),lambda **kw:self.fail('unsafe force'),lambda:True,sleep=lambda _:None)
  self.assertFalse(r['disarmed_confirmed']);self.assertFalse(r['force_disarm_used'])
 def test_ack_without_state_change_uses_bench_stop(self):
  state=[True]
  def force(**kw):state[0]=False;return SimpleNamespace(success=True,result=0)
  r=bench_disarm(lambda **kw:SimpleNamespace(success=True,result=0),force,lambda:state[0],propellers_removed_bench=True,sleep=lambda _:None)
  self.assertTrue(r['force_disarm_used']);self.assertTrue(r['disarmed_confirmed'])
if __name__=='__main__':unittest.main(verbosity=2)
