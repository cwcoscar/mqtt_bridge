import time


while 1:
    nowTime = int(time.time()) # get nowtime
    struct_time = time.localtime(nowTime)# transform to struct_time
    time_stamp = int(time.mktime(struct_time)) # transform to timestamp
    print(time_stamp)