

class Hook():

    def __init__(self) -> None:
        self.lowered = True
        self.hooked = False

    def hook_object(self):
        if (self.hooked):
            self.lowered = False
