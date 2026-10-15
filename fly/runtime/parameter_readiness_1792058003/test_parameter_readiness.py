import unittest
from types import SimpleNamespace as NS
from parameter_readiness import read_required_parameters


class Tests(unittest.TestCase):
    def test_startup_cache_miss_then_success_uses_real_values(self):
        clock=[0.];calls=[]
        def get(param_id):
            calls.append(param_id)
            return NS(success=clock[0]>=.5,value=NS(integer=0,real=1.25))
        result=read_required_parameters(get,['COM_OF_LOSS_T'],lambda:None,clock=lambda:clock[0],
                                       sleep=lambda t:clock.__setitem__(0,clock[0]+t))
        self.assertEqual(result,{'COM_OF_LOSS_T':{'integer':0,'real':1.25}})
        self.assertEqual(calls,['COM_OF_LOSS_T']*2)
    def test_permanently_missing_parameter_times_out_without_default(self):
        clock=[0.]
        with self.assertRaisesRegex(RuntimeError,'超时.*COM_OF_LOSS_T'):
            read_required_parameters(lambda **_:NS(success=False),['COM_OF_LOSS_T'],lambda:None,
                timeout=1,clock=lambda:clock[0],sleep=lambda t:clock.__setitem__(0,clock[0]+t))
        self.assertEqual(clock[0],1.)
    def test_service_failure_retries_then_returns_actual_values(self):
        calls=[]
        def get(**_):
            calls.append(1)
            if len(calls)==1:raise ConnectionError('cache restarting')
            return NS(success=True,value=NS(integer=7,real=0))
        self.assertEqual(read_required_parameters(get,['RC_MAP_FLTMODE'],lambda:None,sleep=lambda _:None),
                         {'RC_MAP_FLTMODE':{'integer':7,'real':0}})
    def test_ground_guard_failure_immediately_aborts_without_read(self):
        def guard():raise RuntimeError('aircraft armed while waiting')
        def get(**_):self.fail('read after guard failure')
        with self.assertRaisesRegex(RuntimeError,'aircraft armed'):
            read_required_parameters(get,['COM_OF_LOSS_T'],guard)
    def test_every_retry_rechecks_previously_read_parameters(self):
        clock=[0.];calls=[]
        def get(param_id):
            calls.append(param_id)
            return NS(success=param_id!='B' or clock[0]>.1,value=NS(integer=int(clock[0]*10),real=0))
        result=read_required_parameters(get,['A','B'],lambda:None,clock=lambda:clock[0],
                                       sleep=lambda t:clock.__setitem__(0,clock[0]+t))
        self.assertEqual(calls,['A','B','A','B']);self.assertEqual(result['A']['integer'],5)


if __name__=='__main__':unittest.main(verbosity=2)
