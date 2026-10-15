import unittest
from record_takeover import diagnose


def e(t,kind,**data):return {'t':t,'kind':kind,'data':data}


class Tests(unittest.TestCase):
    def events(self):return [e(0,'state',armed=True,mode='OFFBOARD'),
        e(1,'state',armed=True,mode='POSCTL'),e(1.2,'extended',landed_state=1),
        e(3.8,'state',armed=False,mode='POSCTL')]
    def test_two_second_landing_param_does_not_fabricate_confirmation(self):
        r=diagnose(self.events(),{'COM_DISARM_LAND':2})
        self.assertEqual(r['status'],'disarm_observed_reason_not_confirmed')
        self.assertIsNone(r['confirmed_disarm_reason'])
    def test_explicit_px4_landing_message_confirms_reason(self):
        r=diagnose(self.events()+[e(3.2,'statustext',text='Disarmed by landing')],{})
        self.assertEqual(r['confirmed_disarm_reason'],'px4_landing_auto_disarm')
    def test_rc_or_command_message_not_misclassified_as_landing(self):
        r=diagnose(self.events()+[e(3.2,'statustext',text='Disarmed by RC (switch)')],{})
        self.assertEqual(r['confirmed_disarm_reason'],'Disarmed by RC (switch)')
    def test_previous_disarm_reason_not_used_for_this_takeover(self):
        r=diagnose([e(-5,'statustext',text='Disarmed by landing')]+self.events(),{})
        self.assertIsNone(r['confirmed_disarm_reason'])


if __name__=='__main__':unittest.main(verbosity=2)
