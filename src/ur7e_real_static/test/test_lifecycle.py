import threading
from types import SimpleNamespace
from unittest.mock import Mock
from ur7e_real_static.node import RealStatic


def test_late_goal_acceptance_after_stop_is_canceled():
    handle=Mock();handle.accepted=True
    future=Mock();future.result.return_value=handle
    node=SimpleNamespace(lock=threading.RLock(),sending=True,generation=4,cancel_requested=True,
                         phase='STOPPING',cancel=Mock(),finished=Mock())
    RealStatic.accepted(node,future,3)
    assert not node.sending and node.goal_handle is handle
    node.cancel.assert_called_once_with('STOPPING')
    assert node.phase!='EXECUTING'


def test_rejected_action_does_not_own_motion():
    handle=Mock();handle.accepted=False
    future=Mock();future.result.return_value=handle
    node=SimpleNamespace(lock=threading.RLock(),sending=True,generation=1,cancel_requested=False,motion_owned=True,phase='SENDING')
    RealStatic.accepted(node,future,1)
    assert node.phase=='EXECUTION_REJECTED' and not node.motion_owned and not node.sending


def test_new_target_cancels_active_action_and_never_starts_parallel_plan():
    node=SimpleNamespace(lock=threading.RLock(),goal_handle=object(),sending=False,generation=1,trajectory=object(),
                         cancel=Mock(),get_logger=lambda:Mock())
    RealStatic.on_goal(node,None)
    assert node.generation==2 and node.trajectory is None
    node.cancel.assert_called_once_with('NEW_GOAL_STOP')


def test_unknown_action_acceptance_latches_unconfirmed_and_blocks_reexecution():
    import pytest
    future=Mock();future.result.side_effect=RuntimeError('connection lost')
    node=SimpleNamespace(lock=threading.RLock(),sending=True,motion_owned=True,goal_handle=None,phase='SENDING')
    RealStatic.accepted(node,future,1)
    assert node.action_uncertain and node.phase=='STOP_UNCONFIRMED' and node.motion_owned
    with pytest.raises(ValueError,match='outstanding'):RealStatic.idle_required(node)
    RealStatic.cancel(node,'USER_STOP')
    assert node.phase=='STOP_UNCONFIRMED' and node.motion_owned


def test_cancellation_result_alone_does_not_release_motion():
    future=Mock();future.result.return_value=SimpleNamespace(status=5)
    node=SimpleNamespace(lock=threading.RLock(),goal_handle=object(),cancel_deadline=1.,
                         cancel_requested=True,motion_owned=True,phase='EXECUTING',trajectory=object())
    RealStatic.finished(node,future)
    assert node.goal_handle is None and node.motion_owned and node.phase=='STOPPING'
    assert node.stop_wait_deadline>0 and node.trajectory is None
