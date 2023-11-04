import paho.mqtt.client as mqtt

intersection0 = None
intersection1 = None

intersections = [intersection0, intersection1]

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("/vtl/request")
    client.subscribe("/vtl/free")

def free_callback(client, userdata, msg):
    global intersections
    if(msg.payload.decode("utf-8") == intersections[0]):
        intersections[0] = None
        print("Intersection 0 is free")
    elif(msg.payload.decode("utf-8") == intersections[1]):
        intersections[1] = None
        print("Intersection 1 is free")
    else:
        print("Error: No intersection with this Car_ID occupied")


def request_callback(client, userdata, msg):
    global intersections
    request = msg.payload.decode("utf-8")
    if(intersections[int(request[1])] == None or intersections[int(request[1])] == request[0]):
        print("publish for intersection" + request[1] + ": " + request[0]+"1")
        # publish car_id, free intersection
        intersections[int(request[1])] = request[0]
        client.publish("/vtl/response", request[0]+"1")
    else:
        print("publish for intersection" + request[1] + ": " + request[0]+"0")
        # publish car_id, occupied intersection
        client.publish("/vtl/response", request[0]+"0")


client = mqtt.Client()
client.on_connect = on_connect
client.message_callback_add("/vtl/request", request_callback)
client.message_callback_add("/vtl/free", free_callback) 



client.connect("localhost", 1883, 60)

client.loop_forever()
