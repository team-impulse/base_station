from gi.repository import  Gtk, GObject, Gdk
import serial,sys,time,os,math,CoordDistance,BNG
from serial.tools.list_ports import comports #import statements

#hard definitions - "constants"
const_default_serial_port = ""
const_default_serial_baud = 38400
const_data_bad_time = 3.0
const_serial_timeout = 0.01
const_lng_mult = -1
const_data_send_time = 1

const_lat=51.000
const_lng=-0.234

#misc
last_data_received_time = 0.0
last_data_sent_time = 0.0
program_start_time = time.time()
class Packet():
    cmd = {}
    def add_cmd(self,c):
        if len(c) > 1:
            cmd_type = c.pop(0)
            self.cmd[cmd_type] = c
        else:
            self.cmd[c[0]] = -1 #set the command type's body to -1 to signify that no byte is to be sent
        print self.cmd

    def send(self):#writes it to be buffered on the Teensy and sent at the next opportunity.
        print "sending packet"
        for cmd_type,cmd_array in self.cmd.iteritems():
            print cmd_type
            serial_port.write(chr(cmd_type))
            if cmd_array != -1:
                for i in cmd_array:
                    serial_port.write(chr(i))#send each command byte.
                    print i

        self.cmd.pop(6,None)
    def wipe(self):
        self.cmd = {}

#next packet to be sent
next_pkt = Packet()

#rover arming states
rvr_motors_armed = False
rvr_para_armed = False
rvr_motors_manual = False

def nameLogFile(base):
  file_counter = 0
  file_name = "data/"+base + str(file_counter) + ".CSV"
  while os.path.isfile(file_name):
    file_counter += 1
    file_name =  "data/"+base + str(file_counter) + ".CSV" ##make sure we don't overwrite anything.

  return file_name

def finaliseSerialWindow(self):#get serial port selection from combo-box; open it.
    cb = builder.get_object("serSelectComboBox")
    ls = cb.get_model()
    if open_serial_port(ls[cb.get_active()][0]):
        #close window and open main window
        windows['sersel'].hide()
        #create and open the main window
        windows['main'] = builder.get_object("MainWindow")
        windows['main'].connect("delete-event",Gtk.main_quit)
        #attach other handlers here.
        GObject.timeout_add(50,update_display,True)#A function to be called every 50ms to update everything
        windows['main'].show_all()#show the main window
    else:
        #ask the user to try another port.
        dialog = Gtk.MessageDialog(self.get_toplevel(), 0, Gtk.MessageType.ERROR, Gtk.ButtonsType.OK, "Couldn't open serial port.")#default message box
        dialog.format_secondary_text(
            "Check the port is closed and connected..")
        dialog.run()#create a blocking dialog box
        dialog.destroy()#get rid of it once all has been finished

def open_serial_port(port_name):#tries to open a specified serial port
    serial_port.port = port_name
    try:
        print("opening",serial_port.port)
        serial_port.timeout = const_serial_timeout #set a timeout for the readline command
        serial_port.open()
        print("Successfully opened serial port.")
        return True
    except (OSError) as e:#if we get a SerialException, the port is
                                                  #probably open already.
        print("Bad serial port name. Try again.")
        print(e)
        return False

def populateSerial():
    cb = builder.get_object("serSelectComboBox")
    ls = Gtk.ListStore(str)
    for p in serial.tools.list_ports.comports():
        f = [p[0]]
        ls.append(f)
    cb.set_model(ls)

def exit_button(self):
    Gtk.main_quit()

def check_serial():
    try:
        return serial_port.readline()
    except:
        return ""

def writeData(item, file):
  file.write(str(item))
  file.write(",")

def nextCounter():
    nc = last_packet_id + 1
    if nc==256:
        nc = 0
    return nc

def process_serial_data(input_data):
    global last_data_received_time #declare that we refer to the global variable. Needed because it in a callback.
    file = open(log_file_names[0],"a")
    file.write(input_data)#this is a file logging the raw data received from the serial port
    file.close()
    print input_data
    sdt = input_data.split(",")#the serial data is comma-separated
    if nextCounter() != int(sdt[24]):
        counts[2]+=1#if we've skipped one in the packet count sequence, add one to the skipped packets counter
    last_packet_id=int(sdt[24])
    if sdt[0]=='PASS':#if CRC has passed
        CRC = True
        counts[0]+=1#received another packet
        last_data_received_time = time.time()
    else:
        counts[0]+=1#received another packet
        counts[1] += 1#and another CRC failure
        CRC = False

    file = open(log_file_names[1],"a")
    if CRC:
        writeData(1,file)
        #RSSI
        RSSI = int(sdt[1])
        writeData(str(RSSI),file)
        data[0].append(RSSI)
        #SNR
        SNR = int(sdt[2])
        data[1].append(SNR)
        writeData(str(SNR),file)
        #pressure
        pressure = ((int(sdt[3])<<8)|(int(sdt[4]))) / 10.0
        data[2].append(pressure)
        writeData(str(pressure),file)
        #height
        ht = (1-pow((pressure/QFE),0.190284))*145366.45
        ht = int(ht)
        data[11].append(ht)
        #external temperature
        ext_temp = ((int(sdt[5])<<8)|(int(sdt[6]))) / 100.0
        data[3].append(ext_temp)
        writeData(str(ext_temp),file)
        #internal temperature
        int_temp = ((int(sdt[7])<<8)|(int(sdt[8]))) / 100.0
        data[4].append(int_temp)
        writeData(str(int_temp),file)
        #humidity
        humidity = ((int(sdt[9])<<8)|int(sdt[10])) / 100.0
        data[5].append(humidity)
        writeData(str(humidity),file)
        #dew point
        dp = data[6][-1]
        if humidity > 0 and humidity <= 100:
            precalc = math.log((humidity/100))+ ((17.62*ext_temp)/(243.12+ext_temp)) #math.log gives ln
            dp = (243.12 * precalc)/(17.62-precalc)#Magnus parameters and formulae from http://irtfweb.ifa.hawaii.edu/~tcs3/tcs3/Misc/Dewpoint_Calculation_Humidity_Sensor_E.pdf
            dp = round(dp,1)
        data[6].append(dp)
        writeData(str(dp),file)
        #GPS:
        gps_lat = ((int(sdt[11])<<24) | (int(sdt[12])<<16) | (int(sdt[13])<<8) | int(sdt[14]))/1000000000.0
        gps_lat+=53
        gps_lng = ((int(sdt[15])<<24) | (int(sdt[16])<<16) | (int(sdt[17])<<8) | int(sdt[18]))/1000000000.0
        if gps_lng < 0.5:
            gps_lng += 1

        gps_lng *= const_lng_mult #make the longitude negative if necessary
        data[7].append(gps_lat)
        data[8].append(gps_lng)
        writeData(gps_lat,file)
        writeData(gps_lng,file)
        #GPS HDOP:
        gps_hdop = float(sdt[23])/10.0
        data[9].append(gps_hdop)
        writeData(gps_hdop,file)
        #course from magnetometer
        crs = (int(sdt[19])<<24) | (int(sdt[20])<<16) | (int(sdt[21])<<8) | int(sdt[22])
        crs /= 100.0
        crs = round(crs,0)
        crs = int(crs)
        writeData(gps_hdop,file)
        writeData((time.time()-program_start_time),file)
        file.write(str(ht))
        file.write('\n')#so write a new line char
        data[10].append(crs)
        data[12].append(time.time()-program_start_time)

    else:
        file.write("0")
        file.write('\n')
    file.close()

def set_data_stats_display():
    data_stat_group = [builder.get_object("data_status_label"),builder.get_object("pkt_cnt_label"),builder.get_object("crc_fail_label"), builder.get_object("snr_label")]
    data_stat_group[1].set_text(str(counts[0]))
    data_stat_group[2].set_text(str(counts[1]))
    data_stat_group[3].set_text(str(data[1][-1]))
    if (time.time()-last_data_received_time)<const_data_bad_time:
        data_stat_group[0].set_markup("<span font-size='xx-large' font-weight='heavy' foreground='#11C000'>Data OK</span>")
    else:
        data_stat_group[0].set_markup("<span font-size='xx-large' font-weight='heavy' foreground='#FF0000'>Data bad</span>")

def check_arm_buttons():
    toggle_button_array = [builder.get_object("motor_arm_toggle"), builder.get_object("para_arm_toggle"), builder.get_object("manual_control_toggle")]
    arm_button_array = [builder.get_object("strut_release_button"), builder.get_object("para_release_button")]
    manual_button_array = [builder.get_object("manual_forward_button"),builder.get_object("manual_right_button"),builder.get_object("manual_back_button"),builder.get_object("manual_left_button")]
    global next_pkt, rvr_motors_armed, rvr_para_armed, rvr_motors_manual
    if toggle_button_array[0].get_active():#Motors armed
        if not rvr_motors_armed:
            arm_button_array[0].set_sensitive(True)
            toggle_button_array[0].set_label("Motors Armed")
            next_pkt.add_cmd([4,255])#command the motors to be armed
        rvr_motors_armed=True
    else:
        arm_button_array[0].set_sensitive(False)
        if rvr_motors_armed:
            toggle_button_array[0].set_label("Motors Disarmed")
            next_pkt.add_cmd([4,0])#disarm motors
            rvr_motors_armed=False

    if toggle_button_array[1].get_active():#Parachute release armed
        arm_button_array[1].set_sensitive(True)
        if not rvr_para_armed:
            toggle_button_array[1].set_label("Parachute Release Armed")
            next_pkt.add_cmd([5])#no way of disarming parachute release at rover end--only UI element.
        rvr_para_armed = True
    else:
        arm_button_array[1].set_sensitive(False)
        if rvr_para_armed:
            toggle_button_array[1].set_label("Parachute Release Disarmed")

    if toggle_button_array[2].get_active() and not rvr_motors_manual:#Manual motor control armed
        for d in manual_button_array:
            d.set_sensitive(True)
        next_pkt.add_cmd([2,1,1])
        rvr_motors_manual = True
    elif not toggle_button_array[2].get_active() and rvr_motors_manual:
        for d in manual_button_array:
            d.set_sensitive(False)
        next_pkt.add_cmd([2,255,255])
        rvr_motors_manual = False

def update_metrics_display():
    labels = [builder.get_object("pressure_label"),builder.get_object("int_temp_label"),builder.get_object("humidity_label"),builder.get_object("ext_temp_label"),builder.get_object("dp_label"),builder.get_object("mag_crs_label"),builder.get_object("height_label"),builder.get_object("qfe_label"),builder.get_object("lat_label"),builder.get_object("lng_label"),builder.get_object("hdop_label"),builder.get_object("distance_label")]
    global data
    labels[0].set_text(str(data[2][-1]))#pressure
    labels[1].set_text(str(data[4][-1]))#MS5637 temp
    labels[2].set_text(str(data[5][-1]))#humidity
    labels[3].set_text(str(data[3][-1]))#HYT271 temperature
    labels[4].set_text(str(data[6][-1]))#calculated dew point
    labels[5].set_text(str(data[10][-1]))#magnetometer heading
    labels[6].set_text(str(data[11][-1]))#height
    labels[7].set_text(str(QFE))#QFE
    labels[8].set_text(str(data[7][-1]))#lat
    labels[9].set_text(str(data[8][-1]))#lng
    labels[10].set_text(str(data[9][-1]))#hdop
    labels[11].set_text(str((CoordDistance.distance_on_unit_sphere(data[7][-1],data[8][-1],const_lat,const_lng) *6373000)))

def update_display(self):
    #function to perform all of the update operations required
    global last_data_sent_time
    if(time.time()-last_data_sent_time)>=const_data_send_time:
        next_pkt.send()
        last_data_sent_time = time.time()
    last_read = check_serial()
    if len(last_read)!=0:#serial data received
         process_serial_data(last_read)
         raw_ser_box = builder.get_object("raw_serial_box")
         ser_buf = raw_ser_box.get_buffer()
         ser_buf.insert(ser_buf.get_end_iter(), last_read)
         raw_ser_box.scroll_mark_onscreen(ser_buf.get_insert())#update raw serial box
         update_metrics_display() #now update the metrics display with the latest data
    set_data_stats_display()#update statistics

    #now to check the arming buttons
    check_arm_buttons()
    return True

def m_f(self):
    next_pkt.add_cmd([2,2,2])#go forwards
def m_b(self):
    next_pkt.add_cmd([2,0,0])#go back
def m_l(self):
    next_pkt.add_cmd([2,1,2])#go left
def m_r(self):
    next_pkt.add_cmd([2,2,1])#go right
def stop_rover(self):
    next_pkt.add_cmd([2,1,1])#stop
def send_waypoint(self):
    wpt_boxes = [builder.get_object("new_wpt_lat_entry"),builder.get_object("new_wpt_lng_entry")]
    new_lat = int(wpt_boxes[0].get_text())
    new_lng = int(wpt_boxes[1].get_text())
    dist = CoordDistance.distance_on_unit_sphere(new_lat,new_lng,data[7][-1],data[8][-1])#8,7
    dist *= 6373000
    print "Travelling ",dist, "m"

def set_qfe(self):
    tb = builder.get_object("qfe_set_box")
    global QFE
    try:
        QFE = float(tb.get_text())
    except ValueError:
        print "invalid QFE value"

counts = [0,0,0] #Packets, CRC failure, missed
last_packet_id = 0
data = [[0],[0],[1013.25],[20.0],[35.0],[20.0],[8.0],[50],[0],[9999],[360],[0],[0.0]]#RSSI,SNR,air pressure,ext temp,int temp, humidity, dew point, lat, long, hdop, course, height,time
QFE = 1013.25
log_file_names = [nameLogFile("RAW_DATA"), nameLogFile("REFINED_DATA")]
serial_port = serial.Serial() #instantiate a new Serial port object
serial_port.baudrate = const_default_serial_baud
windows = {}#a list to hold all of the window objects
builder = Gtk.Builder()
builder.add_from_file("Win1.glade")#import the GUI design file
handler = {
    "exitButtonClicked":exit_button,#assign event handlers
    "serialSelectDone":finaliseSerialWindow,
    "onDeleteWindow":Gtk.main_quit,
    "set_qfe":set_qfe,
    "manual_forward":m_f,
    "manual_back":m_b,
    "manual_left":m_l,
    "manual_right":m_r,
    "rover_stop":stop_rover,
    "send_wpt":send_waypoint,
}
builder.connect_signals(handler)#connect event handlers
populateSerial()#fill the combobox with serial port names
windows['sersel'] = builder.get_object("SerialSelectWindow")#create a new serialselectwindow object
windows['sersel'].show_all()#get the selection.
Gtk.main()#run main loop
