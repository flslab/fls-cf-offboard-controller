import queue
import time
import json


class LiveLogger:
    def __init__(self, file_dir, logger_function=None, limit=20,):  # Buffer 40 frames (at 200Hz, this writes 5 times a sec)
        self.buffer = queue.Queue()
        self.buffer_limit = limit
        self.log_file = open(file_dir, "w")
        self.log_file.write("[\n")
        self.logger_function = logger_function
        self.first_item = True

    def write(self, data):
        if self.logger_function:
            self.logger_function(data)
        self.buffer.put(data)

        # 3. Only write to disk when buffer is full
        if self.buffer.qsize() >= self.buffer_limit:
            self.flush_to_disk()

    def mark_start(self):
        start_entry = {'type': 'start', 'data': time.time()}
        self.write(start_entry)

    def flush_to_disk(self):
        if self.buffer.empty() or not hasattr(self, 'log_file') or self.log_file.closed:
            return

        items_to_write = []

        while not self.buffer.empty():
            try:
                item = self.buffer.get_nowait()
                # We use a comma-prefix if you are building a JSON array manually
                if self.first_item:
                    items_to_write.append(json.dumps(item))
                    self.first_item = False
                else:
                    items_to_write.append(",\n" + json.dumps(item))
                self.buffer.task_done()
            except queue.Empty:
                break

        if items_to_write:
            try:
                self.log_file.write("".join(items_to_write))
                self.log_file.flush()
            except ValueError as e:
                print(f"Failed to write to log: {e}")

    def close(self):
        self.flush_to_disk()  # Save the remaining items
        self.log_file.write("\n]")
        self.log_file.close()