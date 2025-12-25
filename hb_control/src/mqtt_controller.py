import paho.mqtt.client as mqtt
import sys

broker_ip = "localhost"  # Replace with your broker's IP

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker")
        client.subscribe("esp/sensor/#")  # Subscribe to sensor topics
        print("Subscribed to sensor topics")
        # Publish a startup command (optional)
        client.publish("esp/cmd/crystal", "LED_ON", qos=1)
        print("Sent LED_ON command")
    else:
        print(f"Connection failed with code {rc}")
        sys.exit(1)

def on_message(client, userdata, msg):
    print(f"[{msg.topic}] {msg.payload.decode()}")

def on_disconnect(client, userdata, rc):
    print("Disconnected from broker")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect

try:
    client.connect(broker_ip, 1883, 60)
    print("Monitoring started... Press Ctrl+C to stop.")
    client.loop_forever()
except KeyboardInterrupt:
    print("Stopping...")
    client.disconnect()
except Exception as e:
    print(f"Error: {e}")
    client.disconnect()
