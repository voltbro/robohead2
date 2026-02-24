import threading
def run(controller, action_name: str="", cancel_event: threading.Event=None):

    logger = controller.get_logger()
    logger.info(f"[{action_name}] Start")

    controller.media_driver.play_display(cancel_event,
        video_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_left_ear/ears.png",
        loop=True, block=False
    )
    controller.media_driver.play_audio(cancel_event,
        audio_path="/home/pi/robohead_ws/src/robohead2/robohead_controller/robohead_controller/actions/std_left_ear/ears.mp3",
        loop=False, block=False
    )

    for k in range(5):
        result = controller.ears_driver.set_angle(cancel_event, left=90*(-1)**k, right=0, duration=0.5, block=True)

