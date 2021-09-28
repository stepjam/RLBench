class BoundaryError(Exception):
    """Raised when a boundary error occurs."""
    pass


class WaypointError(Exception):
    """Raised when we place a task but cant complete it with path planning."""
    def __init__(self, message, task):
        super().__init__('Error in task %s. %s' % (task.get_name(), message))


class NoWaypointsError(Exception):
    """Raised when there is no waypoints. So cant create demo."""
    def __init__(self, message, task):
        super().__init__('Error in task %s. %s' % (task.get_name(), message))


class DemoError(Exception):
    """Raised when getting demo."""
    def __init__(self, message, task):
        super().__init__('Error in task %s. %s' % (task.get_name(), message))


class InvalidActionError(Exception):
    pass


class TaskEnvironmentError(Exception):
    pass
