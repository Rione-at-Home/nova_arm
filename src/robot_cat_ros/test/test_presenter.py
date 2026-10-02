import unittest
from robot_cat_ros.presenter import PresenterBridge


class Worker:
    def __init__(self):
        self.calls = []

    def submit(self, text, language=None):
        self.calls.append(('submit', text, language))

    def interrupt(self, text=None, language=None):
        self.calls.append(('interrupt', text, language))


class PresenterTests(unittest.TestCase):
    def setUp(self):
        self.worker = Worker()
        self.events = []
        self.bridge = PresenterBridge(self.worker, self.events.append)

    def test_routing(self):
        self.bridge.say('こんにちは')
        self.bridge.say('Hello')
        self.assertEqual(self.worker.calls, [('submit', 'こんにちは', 'ja'),
                                            ('submit', 'Hello', 'en')])

    def test_fault_deduplicates_both_topics_and_recovery_chatter(self):
        self.bridge.health('FAULT')
        self.bridge.presenter_state('ARM_FAULT')
        self.bridge.health('FAULT')
        self.bridge.health('RECOVERING')
        self.bridge.health('FAULT')
        self.assertEqual(self.worker.calls, [('interrupt', 'The robot arm is faulty.', 'en')])
        self.bridge.say('Should not speak')
        self.assertEqual(len(self.worker.calls), 1)
        self.bridge.health('HEALTHY')
        self.bridge.say('Ready')
        self.bridge.health('FAULT')
        self.assertEqual(len(self.worker.calls), 3)

    def test_fault_warning_survives_abort_in_either_order(self):
        self.bridge.abort()
        self.bridge.health('FAULT')
        self.bridge.abort()
        self.assertEqual(self.worker.calls, [('interrupt', None, None),
                                            ('interrupt', 'The robot arm is faulty.', 'en')])

    def test_presenter_fault_first(self):
        self.bridge.presenter_state('ARM_FAULT')
        self.bridge.health('FAULT')
        self.bridge.presenter_state('IDLE')
        self.assertEqual(len(self.worker.calls), 1)
        self.assertTrue(self.bridge.fault_latched)

    def test_override_without_publisher_changes(self):
        bridge = PresenterBridge(self.worker, self.events.append,
                                 text_override='Hello everyone.')
        bridge.say('元の日本語')
        self.assertEqual(self.worker.calls, [('submit', 'Hello everyone.', 'en')])

    def test_other_states_do_not_announce_fault(self):
        for state in ['IDLE', 'DRIVE', 'SCAN', 'SPEAK', 'ARM_PAUSED']:
            self.bridge.presenter_state(state)
        for state in ['HEALTHY', 'DIAGNOSTIC', 'RECOVERING', 'VERIFYING', 'UNKNOWN']:
            self.bridge.health(state)
        self.assertEqual(self.worker.calls, [])


if __name__ == '__main__':
    unittest.main()
