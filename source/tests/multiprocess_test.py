from multiprocessing import Process, Queue
import time

def worker_function(queue):
    while True:
        if not queue.empty():
            item = queue.get()
            print('Got new item from queue:', item)

def main():
    # Create a queue
    queue = Queue()

    # Create a process and pass the queue as an argument
    my_process = Process(target=worker_function, args=(queue,), daemon=True)

    # Start the process
    my_process.start()


    # Retrieve items from the queue in the main process
    i = 0
    while True:
        item = {'val': i}
        queue.put(item)
        time.sleep(3)
        i += 1

if __name__ == "__main__":
    main()