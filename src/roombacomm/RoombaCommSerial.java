/*
 *  RoombaComm Serial Interface
 *
 *  Copyright (c) 2006 Tod E. Kurt, tod@todbot.com, ThingM
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with this library; if not, write to the
 *  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 *  Boston, MA  02111-1307  USA
 *
 */

package roombacomm;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Collections;
import java.util.Map;
import java.util.TreeMap;

import tw.com.prolific.driver.pl2303.PL2303Driver;
import android.content.Context;
import android.hardware.usb.UsbManager;


/**
 *  The serial-port based implementation of RoombaComm.
 *  Handles both physical RS-232 ports, USB adapter ports like Keyspan 
 *  USA-19HS, and Bluetooth serial port profiles.
 * 
 * <p> Some code taken from processing.serial.Serial.  Thanks guys! </p>
 *
 *  The interaction model for setting the port and protocol and WaitForDSR parameters is as follows.
 *  <p>
 *  On creation, the class initializes the parameters, then tries to read .roomba_config.
 *  If it can read the config file and parse out the parameters, it sets the parameters to
 *  the values in the config file. Apps can read the current settings for display using methods
 *  on the class. Apps can override the settings by accepting user input and setting the
 *  parameters using methods on the class, or the connect() method. Parameters that are
 *  changed by the app are re-written in the config file, for use as defaults next run.
 *  Command-line apps can make these parameters optional, by using the defaults if the
 *  user doesn't specify them.  
 */
public class RoombaCommSerial extends RoombaComm
{
    private PL2303Driver.BaudRate rate = PL2303Driver.BaudRate.B57600;
    static final PL2303Driver.DataBits databits = PL2303Driver.DataBits.D8;
    static final PL2303Driver.Parity parity = PL2303Driver.Parity.NONE;
    static final  PL2303Driver.StopBits stopbits = PL2303Driver.StopBits.S1;
    static final PL2303Driver.FlowControl mFlowControl = PL2303Driver.FlowControl.OFF;
    private static final String ACTION_USB_PERMISSION = "com.prolific.pl2303hxdsimpletest.USB_PERMISSION";
    private String protocol = "SCI";

    /**
     * contains a list of all the ports
     * keys are port names (e.g. "/dev/usbserial1")
     * values are Boolean in-use indicator
     */
    static Map ports = null;

    /**
     * The time to wait in milliseconds after sending sensors command before
     * attempting to read
     */
    public static int updateSensorsPause = 400;

    /** The RXTX port object, normally you don't need access to this */
    public PL2303Driver port = null;
    private String    portname = null;   //"/dev/cu.KeySerial1" for instance

    /** 
     * RXTX bombs when flushing output sometimes, so by default do not
     * flush the output stream.  If the output is too buffered to be 
     * useful, do:  
     *  roombacomm.comm.flushOutput = true;
     * before using it and see if it works.
     */
    public boolean flushOutput = false;
    /** 
     * Some "virtual" serial ports like Bluetooth serial on Windows
     * return weird errors deep inside RXTX if an opened port is used
     * before the virtual COM port is ready.  One way to check that it 
     * is ready is to look for the DSR line going high.  
     * However, most simple, real serial ports do not do hardware handshaking
     * so never set DSR high.
     * Thus, if using Bluetooth serial on Windows, do:
     *  roombacomm.waitForDSR = true;
     * before using it and see if it works.
     */
    public boolean waitForDSR = false;		// Warning: public attribute - setting won't trigger config file write

    byte buffer[] = new byte[32768];
    int bufferLast;

    //int bufferSize = 26;  // how big before reset or event firing
    //boolean bufferUntil;
    //int bufferUntilByte;

    /**
     * Let you check to see if a port is in use by another Rooomba
     * before trying to use it.
     */
    public static boolean isPortInUse( String pname ) {
        Boolean inuse = (Boolean) ports.get( pname );
        if( inuse !=null ) {
            return inuse.booleanValue();
        }
        return false;
    }

    // constructor
    public RoombaCommSerial() {
        super();
        makePorts();
        readConfigFile();
    }
    public RoombaCommSerial(boolean autoupdate) {
        super(autoupdate);
        makePorts();
        readConfigFile();
    }
    public RoombaCommSerial(boolean autoupdate, int updateTime) {
        super(autoupdate, updateTime);
        makePorts();
        readConfigFile();
    }

    void makePorts() {
        if( ports == null ) 
            ports = Collections.synchronizedMap(new TreeMap());
    }
    /**
     * Connect to a serial port specified by portid
     * doesn't guarantee connection to Roomba, just to serial port
     * @param portid name of port, e.g. "/dev/cu.KeySerial1" or "COM3"
     * @return true if connect was successful, false otherwise
     */
    public boolean connect(UsbManager usbManager, Context context) {

        connected = open_port(usbManager, context);

        if( connected ) {
            // log in the global ports hash if the port is in use now or not
         //   ports.put( portname, new Boolean( connected ) );
            sensorsValid = false;
        }
        else {
            disconnect();
        }
        
        return connected;
    }

    /**
     * Disconnect from serial port
     */
    public void disconnect() {
        connected = false;

        // log in the global ports hash if the port is in use now or not    
        try {
            if (port != null) port.end();  // close the port 
        } catch (Exception e) {
            e.printStackTrace();
        }
        port = null;
    }

    /**
     * subclassed.  FIXME: 
     */
    public boolean send(byte[] bytes) {
        try {
            port.write(bytes);
        } catch (Exception e) { // null pointer or serial port dead
            e.printStackTrace();
        }
        return true;
    }

    /**
     * This will handle both ints, bytes and chars transparently.
     */
    public boolean send(int b) {  // will also cover char or byte
        try {
            port.write(new byte[]{(byte)(b & 0xff)});  // for good measure do the &
        } catch (Exception e) { // null pointer or serial port dead
            //errorMessage("send", e);
            e.printStackTrace();
        }
        return true;
    }

    /**
     * toggles DD line via serial port DTR  (if available)
     */
    public void wakeup() {
    //    port.setDTR(false);
    //    pause(500);
    //    port.setDTR(true);
    }
  
    /**
     * Update sensors.  Block for up to 1000 ms waiting for update
     * To use non-blocking, call sensors() and then poll sensorsValid()
     */
    public boolean updateSensors() {
        sensorsValid = false;
        sensors();
        for(int i=0; i < 20; i++) {
            if( sensorsValid ) { 
                logmsg("updateSensors: sensorsValid!");
                break;
            }
            logmsg("updateSensors: pausing...");
            pause( 50 );
        }

        return sensorsValid;
    }
    
    /**
     * Update sensors.  Block for up to 1000 ms waiting for update
     * To use non-blocking, call sensors() and then poll sensorsValid()
     */
    public boolean updateSensors(int packetcode) {
        sensorsValid = false;
        sensors(packetcode);
        for(int i=0; i < 20; i++) {
            if( sensorsValid ) { 
                logmsg("updateSensors: sensorsValid!");
                break;
            }
            logmsg("updateSensors: pausing...");
            pause( 50 );
        }

        return sensorsValid;
    }

    /**
     * called by serialEvent when we have enough bytes to make sensors valid
     */
    public void computeSensors() {
        sensorsValid = true;
        sensorsLastUpdateTime = System.currentTimeMillis();
        computeSafetyFault();
    }
        /*
        pause(updateSensorsPause);     // take a breather to let data come back
        sensorsValid = false;          // assume the worst, we're gothy
        int n = available();
        //logmsg("updateSensors:n="+n);
        if( n >= 26) {                 // there are enough bytes to read
            n = readBytes(sensor_bytes);
            if( n==26 ) {              // did we get enough?
                sensorsValid = true;   // then everything's good, otherwise bad
                computeSafetyFault();
            }
        } else {
            logmsg("updateSensors:only "+n+" bytes available, not updating sensors");
        }
        
        //logmsg("buffer contains: "+ buffer );
        return sensorsValid;
        */

    /**
     * If this just hangs and never completes on Windows,
     * it may be because the DLL doesn't have its exec bit set.
     * Why the hell that'd be the case, who knows.
     * FIXME: deal more gracefully
     * (from processing.serial.Serial)
     */
 /*   public String[] listPorts() {
        Map ps = Collections.synchronizedMap(new LinkedHashMap());
        //Vector list = new Vector();
        try {
            //System.err.println("trying");
            Enumeration portList = CommPortIdentifier.getPortIdentifiers();
            //System.err.println("got port list");
            while (portList.hasMoreElements()) {
                CommPortIdentifier portId =
                    (CommPortIdentifier) portList.nextElement();
                logmsg("Found port: " + portId.getName());
                
                if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
                    String name = portId.getName();
                    //list.addElement(name);
                    Boolean state = (Boolean) ports.get( name );
                    if( state==null ) state = new Boolean(false);
                    ps.put( name, state );
                }
            }
        } catch (UnsatisfiedLinkError e) {
            //System.err.println("1");
            errorMessage("listPorts", e);
        } catch (Exception e) {
            //System.err.println("2");
            errorMessage("listPorts", e);
        }
        //System.err.println("move out");
        /*
        for( Enumeration e = list.elements(); e.hasMoreElements(); ) {
            String p = (String) e.nextElement();
            if( ! ports.containsKey( p ) ) {
                ports.put( p, new Boolean(false) );
            }
        }

        // DEBUG
        System.err.println("ports hashtable:");
        for( Enumeration e = ports.keys(); e.hasMoreElements(); ) {
            String p = (String) e.nextElement();
            Boolean b = (Boolean) ports.get(p);
            System.err.println("port:"+p+", inuse:"+b);
        }
        */ 
   /*     ports = ps;
        String outgoing[] = 
            (String[]) new TreeSet(ports.keySet()).toArray(new String[0]);

        return outgoing;
    } */


    public String getProtocol() {
		return protocol;
	}

	public void setProtocol(String protocol) {
		if (protocol.equals("SCI")) {
			rate = PL2303Driver.BaudRate.B57600;
		} else if (protocol.equals("OI")) {
			rate = PL2303Driver.BaudRate.B115200;
		}
		this.protocol = protocol;
		logmsg("Protocol: " + protocol);
		writeConfigFile(portname, protocol, waitForDSR?'Y':'N');
	}

	public boolean isWaitForDSR() {
		return waitForDSR;
	}

	public void setWaitForDSR(boolean waitForDSR) {
		this.waitForDSR = waitForDSR;
		writeConfigFile(portname, protocol, waitForDSR?'Y':'N');
	}

	public String getPortname() {
		return portname;
	}
	
	public void setPortname(String p) {
		portname = p;
		logmsg("Port: " + portname);
		writeConfigFile(portname, protocol, waitForDSR?'Y':'N');

	}

   // -------------------------------------------------------------
    // below only used internally to this class
    // -------------------------------------------------------------

	/**
     * internal method, used by connect()
     * FIXME: make it faile more gracefully, recognize bad port
     */
    private boolean open_port(UsbManager usbManager, Context context) {
        boolean success = false;
        try {                
                port = new PL2303Driver(usbManager,
                        context, ACTION_USB_PERMISSION);
                if( !port.enumerate() ) {
                	System.out.println("error");
                }
                port.setup(rate, databits, stopbits, parity, mFlowControl);
                port.InitByBaudRate(rate);

                logmsg("port "+portname+" opened successfully");
                success = true;
        } catch (Exception e) {
            logmsg("connect failed: "+e);
            port = null;
        }
                        
        return success;
    }
    
    /**
     * Write a config file with current settings
     */
    private void writeConfigFile(String port, String protocol, char waitForDSR)
    {
    	try {
    		FileWriter f = new FileWriter(".roomba_config", false);
        	BufferedWriter w = new BufferedWriter(f); // create file
        	w.write(port);
        	w.newLine();
        	w.write(protocol);
        	w.newLine();
        	w.write(waitForDSR);
        	w.newLine();
        	w.close();
        	f.close();
    	} catch (IOException e) {
    		logmsg("unable to write .roomba_config " + e);
    	}
    }
    private void readConfigFile()
    {
    	try {
    		FileReader f = new FileReader(".roomba_config");
    		BufferedReader r = new BufferedReader(f);
    		portname = r.readLine();
    		protocol = r.readLine();
    		if (protocol.equals("OI")) {
    			rate = PL2303Driver.BaudRate.B115200;
    		}
    		waitForDSR = r.readLine().equals("Y")?true:false;
    		logmsg("read config port: " + port + " protocol: " + protocol + " waitDSR: " + waitForDSR);    		
    	} catch (IOException e) {
    		logmsg("unable to read .roomba_config " + e);
    	}
    }

    /**
     * Returns the number of bytes that have been read from serial
     * and are waiting to be dealt with by the user.
     * (from processing.serial.Serial)
     *
    private int available() {
        return (bufferLast - bufferIndex);
    }

    /**
     * Return a byte array of anything that's in the serial buffer.
     * Not particularly memory/speed efficient, because it creates
     * a byte array on each read, but it's easier to use than
     * readBytes(byte b[]) (see below).
     * (from processing.serial.Serial)
     *
    private byte[] readBytes() {
        if (bufferIndex == bufferLast) return null;

        synchronized (buffer) {
            int length = bufferLast - bufferIndex;
            byte outgoing[] = new byte[length];
            System.arraycopy(buffer, bufferIndex, outgoing, 0, length);

            bufferIndex = 0;  // rewind
            bufferLast = 0;
            return outgoing;
        }
    }
  
    /**
     * Grab whatever is in the serial buffer, and stuff it into a
     * byte buffer passed in by the user. This is more memory/time
     * efficient than readBytes() returning a byte[] array.
     *
     * Returns an int for how many bytes were read. If more bytes
     * are available than can fit into the byte array, only those
     * that will fit are read.
     * (from processing.serial.Serial)
     *
    public int readBytes(byte outgoing[]) {
        if (bufferIndex == bufferLast) return 0;

        synchronized (buffer) {
            int length = bufferLast - bufferIndex;
            if (length > outgoing.length) length = outgoing.length;
            System.arraycopy(buffer, bufferIndex, outgoing, 0, length);

            bufferIndex += length;
            if (bufferIndex == bufferLast) {
                bufferIndex = 0;  // rewind
                bufferLast = 0;
            }
            return length;
        }
    }
    */

}
