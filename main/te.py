from api_client import RuntimeApiClient

client = RuntimeApiClient()

client.call("car", "beep", timeout=40)
client.call("car", "move_for", [0.05, 0.0, 0.0], timeout=90)
client.call("arm", "move_x_position", 0.20, timeout=20)