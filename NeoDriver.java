package org.firstinspires.ftc.teamcode.util.drivers;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "NeoDriver", xmlTag = "NeoDriver")
public class NeoDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    byte ADDRESS_I2C_DEFAULT = 0x60;
    byte moduleBase = 0x0E;
    public static int pixels = 160;


    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    protected boolean doInitialize() {
        if (deviceClient instanceof LynxI2cDeviceSynch) {
            ((LynxI2cDeviceSynch) deviceClient).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        } else {
            throw new RuntimeException("Fuck fuck");
        }

        // NeoDriver board routes NEO signal to seesaw pin 15 : contentReference[oaicite:3]{index=3}
        write8(FunctionRegister.PIN, (byte) 15);

        // 800 kHz NeoPixel timing (SPEED = 0x01) :contentReference[oaicite:4]{index=4}
        write8(FunctionRegister.SPEED, (byte) 0x01);

        // Buffer length in bytes = pixels * 3 (RGB), little-endian :contentReference[oaicite:5]{index=5}
        int bufLen = pixels * 3;  // 160 * 3 = 480
        write16BE(FunctionRegister.BUF_LENGTH, bufLen);

        return true;
    }

    @Override
    public String getDeviceName() {
        return "Adafruit NeoDriver";
    }

    protected void writeShort(final FunctionRegister reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    public void writeColorAll(byte r, byte g, byte b) {
        final int bytesPerPixel = 3;
        int bufLen = pixels * bytesPerPixel;

        // Full GRB frame
        byte[] frame = new byte[bufLen];
        int ptr = 0;
        for (int i = 0; i < pixels; i++) {
            frame[ptr++] = g;
            frame[ptr++] = r;
            frame[ptr++] = b;
        }

        int offset = 0;
        while (offset < bufLen) {
            int chunk = Math.min(30, bufLen - offset); // max 30 bytes per write

            // [function][startLo][startHi][data...]
            byte[] payload = new byte[1 + 2 + chunk];
// [function][startHi][startLo][data...]
            payload[0] = (byte) FunctionRegister.BUF.bVal;   // 0x04

            payload[1] = (byte) ((offset >> 8) & 0xFF);      // startHi
            payload[2] = (byte) (offset & 0xFF);

            System.arraycopy(frame, offset, payload, 3, chunk);

            deviceClient.write(moduleBase, payload);
            offset += chunk;
        }

        show();
    }

    public void turnOff() {
        final int bytesPerPixel = 3;
        int bufLen = pixels * bytesPerPixel; // total bytes in NeoPixel buffer

        int offset = 0;
        while (offset < bufLen) {
            int chunk = Math.min(30, bufLen - offset); // max 30 data bytes

            // Payload layout for BUF:
            // [function][startLo][startHi][data...]
            byte[] payload = new byte[1 + 2 + chunk];
// [function][startHi][startLo][data...]
            payload[0] = (byte) FunctionRegister.BUF.bVal;   // 0x04

            payload[1] = (byte) ((offset >> 8) & 0xFF);      // startHi
            payload[2] = (byte) (offset & 0xFF);

            // data bytes 3..end are left as 0 (Java default) -> all LEDs = (0,0,0)

            deviceClient.write(moduleBase, payload);
            offset += chunk;
        }

        // SHOW command: [addr][0x0E][0x05]
        show();
    }

    protected void write(FunctionRegister reg, byte[] data) {
        // builds [functionRegister][data...] and sends with moduleBase as the first register
        byte[] buf = new byte[1 + data.length];
        buf[0] = (byte) reg.bVal;
        System.arraycopy(data, 0, buf, 1, data.length);

        // First register byte on the wire is moduleBase (0x0E for NeoPixel)
        deviceClient.write(moduleBase, buf);
    }

    // 8-bit write (PIN, SPEED, etc.)
    protected void write8(FunctionRegister reg, byte value) {
        write(reg, new byte[]{ value });
    }

    // 16-bit little-endian write (BUF_LENGTH, etc.)
    protected void write16LE(FunctionRegister reg, int value) {
        byte[] le = new byte[2];
        le[0] = (byte) (value & 0xFF);         // low byte
        le[1] = (byte) ((value >> 8) & 0xFF);  // high byte
        write(reg, le);
    }

    // 16-bit big-endian write (for NeoPixel BUF_LENGTH)
    protected void write16BE(FunctionRegister reg, int value) {
        byte[] be = new byte[2];
        be[0] = (byte) ((value >> 8) & 0xFF); // high byte
        be[1] = (byte) (value & 0xFF);        // low byte
        write(reg, be);
    }

    // command with no data (SHOW)
    protected void sendCommand(FunctionRegister reg) {
        deviceClient.write(moduleBase, new byte[]{ (byte) reg.bVal });
    }

    public void show() {
        // SHOW has no args, just the function register :contentReference[oaicite:6]{index=6}
        sendCommand(FunctionRegister.SHOW);
    }

    public NeoDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(I2cAddr.create7bit(ADDRESS_I2C_DEFAULT & 0xFF));
        doInitialize();
        super.registerArmingStateCallback(false);
        // all real setup now happens in doInitialize()
    }

    public enum FunctionRegister
    {
        FIRST(0),
        PIN(0x01),
        SPEED(0x02),
        BUF_LENGTH(0x03),
        BUF(0x04),
        SHOW(0x05),
        LAST(SHOW.bVal);

        public int bVal;

        FunctionRegister(int bVal)
        {
            this.bVal = bVal;
        }
    }
}
