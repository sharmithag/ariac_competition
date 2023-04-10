from ariac_msgs.msg import AssemblyTask, KittingTask, CombinedTask
class Orders:


    def __init__(self, id, priority, type, task):
        self.orders = {}
        # order_object = []   
        self.id = id
        self.priority=priority
        self.type=type

        self.task=task
  

class Parts:
    def __init__(self):
        # " 0 " - Bin Parts, " 1 " - Conveyor Parts
        self.partsDict = {"bin" :[], "conveyor":[]}
    def print_dict(self):
        return self.partsDict