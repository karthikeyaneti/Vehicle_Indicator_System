import serial
from datetime import datetime
import time

def safe_serial_read(ser, max_retries=3):
    """Attempt to read and decode a line with error handling"""
    for attempt in range(max_retries):
        try:
            # Read raw bytes
            raw_data = ser.readline()
            
            # Skip empty reads
            if not raw_data:
                return None
                
            # Try UTF-8 decode first
            try:
                return raw_data.decode('utf-8').strip()
            except UnicodeDecodeError:
                # Fallback to error-resistant decoding
                return raw_data.decode('utf-8', errors='replace').strip().replace('\x00', '')
                
        except Exception as e:
            print(f"Read attempt {attempt+1} failed: {str(e)}")
            time.sleep(0.1)
    return None

def main():
    try:
        ser = serial.Serial('COM9', 115200, timeout=1)
        
        with open("indicator_log.csv", "a", encoding='utf-8') as log_file:
            # Write header if new file
            if log_file.tell() == 0:
                log_file.write("Timestamp,DateTime,Event,LeftLED,RightLED,Hazard,LeftButton,RightButton\n")
            
            print("Logging started. Press Ctrl+C to stop.")
            
            while True:
                line = safe_serial_read(ser)
                if line:
                    try:
                        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                        log_file.write(f"{line[:line.find(',')]},{now},{line[line.find(',')+1:]}\n")
                        log_file.flush()
                        print(line)  # Console echo
                    except Exception as e:
                        print(f"Logging error: {e}")
                        
    except KeyboardInterrupt:
        print("\nLogging stopped by user")
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print("Resources cleaned up")

if __name__ == "__main__":
    main()