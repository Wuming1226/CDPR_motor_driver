from scipy.spatial.transform import Rotation as R

'''
tranformation between eulers and quaternion
param 'axis': rotation sequence ('xyz')
'''
def quaternion2euler(axis, quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler(axis, degrees=True)
    return euler

def euler2quaternion(axis, euler):
    r = R.from_euler(axis, euler, degrees=True)
    quaternion = r.as_quat()
    return quaternion

'''
relist an int to SDO message data form
'''
def relist(num):
    
    if num > 0:
    
      num = int(num)
	    
      a = hex((num & 0xFF))
      b = num & 0xFF00
      c = num & 0xFF0000
      d = num & 0xFF000000
	    
      b = hex((b >> 8))
      c = hex((c >> 16))
      d = hex((d >> 24))
      
	    
      a = int(a,16)
      b = int(b,16)
      c = int(c,16)
      d = int(d,16)
      
      return a, b, c, d

    else:
      num = int(num)
      inv = int(hex(num & 0xFFFFFFFF), 16)
      a = inv & 0xFF
      b = inv & 0xFF00
      c = inv & 0xFF0000
      d = inv & 0xFF000000
      
      b = hex(b>>8)
      c = hex(c>>16) 
      d = hex(d>>24)
      
      b = int(b,16)
      c = int(c,16)
      d = int(d,16)
      
      return a, b, c, d


'''
decode SDO message and extract data in int
'''
def decode(msg):

    data = hex(msg[7]<<24 | msg[6]<<16 | msg[5]<<8 | msg[4])

    if msg[7] == 255:      # if data < 0
        result = int(str(data[4:]), 16)
        if result & (1 << 23):
            result -= 1 << 24
    else:                   # else data > 0 
        result = int(data, 16)
    return result

