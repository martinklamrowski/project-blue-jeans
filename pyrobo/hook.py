class Hook():
    def __init__(self) -> None:
        self.lowered = True
        self.hooked = False

    def hook_object(self):
        self.hooked = True
        self.lowered = False

    def unhook_object(self):
        self.hooked = False
        self.lowered = True