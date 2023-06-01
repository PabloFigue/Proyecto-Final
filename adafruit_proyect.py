# Import necessary libraries
import serial
import struct
import time
from Adafruit_IO import Client, Feed, MQTTClient

# Set to your Adafruit IO key and username.
ADAFRUIT_IO_KEY = "aio_SCMx91JTBdB9SazOIfBRVbA2FCk0"
ADAFRUIT_IO_USERNAME = "PabloFig"

# Set the ID of the feeds to send and subscribe to for updates.
S1_FEED_ID = "servo1"
S2_FEED_ID = "servo2"
S3_FEED_ID = "servo3"
S4_FEED_ID = "servo4"
SAVE_FEED_ID = "salvar"

# Set the COM port for serial communication
SERIAL_PORT = 'COM5'

# Create an instance of the Adafruit IO REST client.
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Create feeds if they don't exist
try:
    aio.feeds(S1_FEED_ID)
except:
    feed = Feed(name=S1_FEED_ID)
    aio.create_feed(feed)

try:
    aio.feeds(S2_FEED_ID)
except:
    feed = Feed(name=S2_FEED_ID)
    aio.create_feed(feed)

try:
    aio.feeds(S3_FEED_ID)
except:
    feed = Feed(name=S3_FEED_ID)
    aio.create_feed(feed)
    
try:
    aio.feeds(S4_FEED_ID)
except:
    feed = Feed(name=S4_FEED_ID)
    aio.create_feed(feed)

try:
    aio.feeds(SAVE_FEED_ID)
except:
    feed = Feed(name=SAVE_FEED_ID)
    aio.create_feed(feed)

# Initialize the serial connection
ser = serial.Serial(SERIAL_PORT, 9600,timeout=1)
print("Serial port", SERIAL_PORT, "opened")

# Set the initial value of the modificador feed to 0
aio.send_data(S1_FEED_ID, 20)
aio.send_data(S2_FEED_ID, 20)
aio.send_data(S3_FEED_ID, 20)
aio.send_data(S4_FEED_ID, 20)
aio.send_data(SAVE_FEED_ID, "OFF")

# Define callback functions for the MQTT client.
def connected(client):
    print("Subscribing to Feed {0}".format(S1_FEED_ID))
    client.subscribe(S1_FEED_ID)
    print("Subscribing to Feed {0}".format(S2_FEED_ID))
    client.subscribe(S2_FEED_ID)
    print("Subscribing to Feed {0}".format(S3_FEED_ID))
    client.subscribe(S3_FEED_ID)
    print("Subscribing to Feed {0}".format(S4_FEED_ID))
    client.subscribe(S4_FEED_ID)
    print("Subscribing to Feed {0}".format(SAVE_FEED_ID))
    client.subscribe(SAVE_FEED_ID)
    print("Waiting for feed data...")

def disconnected(client):
    sys.exit(1)

def message(client, feed_id, payload):
    print("Feed {0} received new value: {1}".format(feed_id, payload))

# Create an MQTT client instance.
mqtt_client = MQTTClient(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Setup the callback functions defined above.
mqtt_client.on_connect = connected
mqtt_client.on_disconnect = disconnected
mqtt_client.on_message = message

# Connect to the Adafruit IO server.
mqtt_client.connect()

# Start the MQTT client's background thread to listen for messages.
mqtt_client.loop_background()

# Variables to hold the count for the feeds
value_modificador1 = int(aio.receive(S1_FEED_ID).value)
value_modificador2 = int(aio.receive(S2_FEED_ID).value)
value_modificador3 = int(aio.receive(S3_FEED_ID).value)
value_modificador4 = int(aio.receive(S4_FEED_ID).value)
value0 = 0
value1 = 1
value2 = 2
value3 = 3
value4 = 4
value5 = 255

# Main loop
while True:

    # Check for any changes in the modificador feed value
    
    modificador1 = int(aio.receive(S1_FEED_ID).value)
    modificador2 = int(aio.receive(S2_FEED_ID).value)
    modificador3 = int(aio.receive(S3_FEED_ID).value)
    modificador4 = int(aio.receive(S4_FEED_ID).value)
    modificadorS = aio.receive(SAVE_FEED_ID).value
    
    if modificador1 != value_modificador1:
        value_modificador1 = modificador1
        binary_data = struct.pack('B', value1)
        ser.write(binary_data)
        time.sleep(1)
        # Convertir el valor a formato binario (1 byte)
        binary_data = struct.pack('B', modificador1)
        # Enviar los datos binarios al puerto serial
        ser.write(binary_data)
        print("Modificador servo1 set to", modificador1)
    elif modificador2 != value_modificador2:
        value_modificador2 = modificador2
        binary_data = struct.pack('B', value2)
        ser.write(binary_data)
        time.sleep(1)
        binary_data = struct.pack('B', modificador2)
        ser.write(binary_data)
        print("Modificador servo2 set to", modificador2)
    elif modificador3 != value_modificador3:
        value_modificador3 = modificador3
        binary_data = struct.pack('B', value3)
        ser.write(binary_data)
        time.sleep(1)
        binary_data = struct.pack('B', modificador3)
        ser.write(binary_data)
        print("Modificador servo3 set to", modificador3)
    elif modificador4 != value_modificador4:
        value_modificador4 = modificador4
        binary_data = struct.pack('B', value4)
        ser.write(binary_data)
        time.sleep(1)
        binary_data = struct.pack('B', modificador4)
        ser.write(binary_data)
        print("Modificador servo4 set to", modificador4)
    time.sleep(1)
    if modificadorS == "OFF":
        binary_data = struct.pack('B', value0)
        ser.write(binary_data)
    elif modificadorS == "ON":
        binary_data = struct.pack('B', value5)
        ser.write(binary_data)

    # Wait for a short period of time
    #time.sleep(3)



