from scene import resolve


def move_to(target):
    pos = resolve(target)
    print("Moving to", pos)


def grasp(target):
    print("Grasping", target)


def lift(height):
    print("Lifting", height)


def place(target):
    pos = resolve(target)
    print("Placing at", pos)
