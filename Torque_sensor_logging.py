import time
from gpiozero import MCP3008
import csv
from datetime import datetime

# Get the current date and time to format the log filename
current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_filename = f'adc_readings_{current_time}.csv'

with open(log_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    
    writer.writerow(['Timestamp', 'ADC Value'])
    
    # Create an object for MCP3008 on channel 0
    adc = MCP3008(channel=0)
    
    try:
        while True:
            # Get the current timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            value = adc.value
            
            # Write the timestamp and ADC value to the CSV file
            writer.writerow([timestamp, value])
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Logging stopped")
    
    finally:
        print(f"Program finished. Data logged to {log_filename}.")
