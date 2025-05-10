import csv

def init_logger():
    log_file = open("log.csv", "w", newline='')
    log_writer = csv.writer(log_file)
    log_writer.writerow(["Time", "Edges", "Lines", "PathLen", "Distance", "Command", "ArduinoResponse"])
    return log_writer, log_file

def log_entry(writer, timestamp, edges, lines, pathlen, distance, cmd, arduino_reply):
    writer.writerow([
        timestamp,
        edges,
        lines,
        pathlen,
        distance,
        cmd,
        arduino_reply
    ])
