scene = {
    "mug": {
        "body": [0.7, 0, 0.1],
        "handle": [0.7, 0.05, 0.15],
    }
}


def resolve(target):
    obj, part = target.split(".")
    return scene[obj][part]