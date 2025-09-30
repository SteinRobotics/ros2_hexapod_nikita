from pyjoystick.sdl2 import Joystick, run_event_loop

def print_add(joy):
    print(f"Joystick hinzugefügt: {joy}")

def print_remove(joy):
    print(f"Joystick entfernt: {joy}")

def handle_event(event):
    if event.type == 'axis':
        print(f"Achse {event.axis}: {event.value:.2f}")
    elif event.type == 'button':
        if event.pressed:
            print(f"Taste {event.button} gedrückt!")
        else:
            print(f"Taste {event.button} losgelassen!")

# Startet die Event-Schleife
run_event_loop(
    add_joystick=print_add,
    remove_joystick=print_remove,
    handle_key_event=handle_event
)
