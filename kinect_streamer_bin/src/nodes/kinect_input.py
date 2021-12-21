import keyboard  # using module keyboard

recorded = keyboard.record(until='esc')
for key in recorded:
    print(key)