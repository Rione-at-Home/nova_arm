"""Presenter integration policy, independent of ROS for unit testing."""
import re


class PresenterBridge:
    def __init__(self, worker, report, language='auto', text_override='',
                 fault_text='The robot arm is faulty.', fault_language='en'):
        if language not in ('auto', 'en', 'ja', 'current'):
            raise ValueError('presenter_language must be auto, en, ja or current')
        if fault_language not in ('en', 'ja'):
            raise ValueError('fault_language must be en or ja')
        if not fault_text.strip() or len(fault_text) > 1000:
            raise ValueError('fault_text must contain 1-1000 characters')
        if len(text_override) > 1000:
            raise ValueError('presenter_text_override must be at most 1000 characters')
        self.worker, self.report = worker, report
        self.language, self.text_override = language, text_override
        self.fault_text, self.fault_language = fault_text, fault_language
        self.fault_latched = False

    def say(self, text):
        if self.fault_latched:
            self.report('presenter:blocked_arm_fault')
            return
        if not text.strip():
            return
        text = self.text_override or text
        language = self.language
        if language == 'auto':
            language = 'ja' if re.search(r'[\u3040-\u30ff\u3400-\u9fff\uff66-\uff9f]', text) else 'en'
        self.worker.submit(text, None if language == 'current' else language)

    def health(self, state):
        state = state.strip().upper()
        if state == 'HEALTHY':
            was_faulted = self.fault_latched
            self.fault_latched = False
            if was_faulted:
                self.report('arm:healthy')
        elif state in ('FAULT', 'ARM_FAULT'):
            self.fault()
        # DIAGNOSTIC, RECOVERING and VERIFYING do not rearm the warning.
        # The presenter coordinator owns movement and health timeout policy.

    def presenter_state(self, state):
        if state.strip().upper() == 'ARM_FAULT':
            self.fault()

    def fault(self):
        if self.fault_latched:
            return
        self.fault_latched = True
        self.worker.interrupt(self.fault_text, self.fault_language)
        self.report('arm:fault_warning_queued')

    def abort(self):
        if self.fault_latched:
            # Coordinator may publish abort and fault almost simultaneously.
            # Never let that abort erase the pending/active fault warning.
            self.report('presenter:aborted_fault_warning_preserved')
        else:
            self.worker.interrupt()
            self.report('presenter:aborted')
