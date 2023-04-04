from ariac_msgs.msg import AssemblyTask, KittingTask, CombinedTask
class Orders:
    def __init__(self, id, priority, type, task):
        self.orders = {}
        # order_object = []   
        self.id = id
        self.priority=priority
        self.type=type

        self.task=task