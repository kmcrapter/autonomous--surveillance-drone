"""Autonomous Mission Finite State Machine."""

from enum import Enum
from typing import Set

class MissionState(Enum):
    IDLE = "IDLE"
    PRECHECK = "PRECHECK"
    TAKEOFF = "TAKEOFF"
    SURVEYING = "SURVEYING"
    PAUSED = "PAUSED"
    RETURNING = "RETURNING"
    LANDING = "LANDING"
    CHARGING = "CHARGING"
    RESUMING = "RESUMING"
    COMPLETE = "COMPLETE"
    DEGRADED = "DEGRADED"
    ABORTED = "ABORTED"

class MissionStateMachine:
    """Enforces valid state transitions and tracks state timing."""

    VALID_TRANSITIONS = {
        MissionState.IDLE: {MissionState.PRECHECK, MissionState.ABORTED},
        MissionState.PRECHECK: {MissionState.TAKEOFF, MissionState.IDLE, MissionState.ABORTED},
        MissionState.TAKEOFF: {MissionState.SURVEYING, MissionState.RETURNING, MissionState.ABORTED},
        MissionState.SURVEYING: {MissionState.PAUSED, MissionState.RETURNING, MissionState.DEGRADED, MissionState.COMPLETE, MissionState.ABORTED},
        MissionState.PAUSED: {MissionState.SURVEYING, MissionState.RETURNING, MissionState.ABORTED},
        MissionState.RETURNING: {MissionState.LANDING, MissionState.ABORTED},
        MissionState.LANDING: {MissionState.CHARGING, MissionState.COMPLETE, MissionState.IDLE, MissionState.ABORTED},
        MissionState.CHARGING: {MissionState.RESUMING, MissionState.IDLE, MissionState.COMPLETE},
        MissionState.RESUMING: {MissionState.TAKEOFF, MissionState.SURVEYING, MissionState.COMPLETE, MissionState.ABORTED},
        MissionState.COMPLETE: {MissionState.IDLE, MissionState.PRECHECK},
        MissionState.DEGRADED: {MissionState.SURVEYING, MissionState.RETURNING, MissionState.ABORTED},
        MissionState.ABORTED: {MissionState.LANDING, MissionState.IDLE}
    }

    def __init__(self, initial_state: MissionState = MissionState.IDLE):
        self._current_state = initial_state
        self._previous_state = None
        self._state_history = [(initial_state, 0.0)]

    @property
    def current_state(self) -> MissionState:
        return self._current_state

    @property
    def previous_state(self) -> MissionState:
        return self._previous_state

    def transition_to(self, new_state: MissionState, timestamp: float) -> bool:
        if new_state == self._current_state:
            return True
        allowed = self.VALID_TRANSITIONS.get(self._current_state, set())
        if new_state in allowed or new_state in (MissionState.ABORTED, MissionState.IDLE):
            self._previous_state = self._current_state
            self._current_state = new_state
            self._state_history.append((new_state, timestamp))
            return True
        return False
