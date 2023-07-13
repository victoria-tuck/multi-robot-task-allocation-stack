#!/usr/bin/env python

def read_map(name):
    map = open(name+'.pgm','rb')
    new_map = open(name+'_inv.pgm','wb')
    assert map.readline() == b'P5\n'
    while(1):
      line = map.readline()
      if line[0] != 35:
        break
    (width, height) = [int(i) for i in line.split()]
    depth = int(map.readline())
    assert depth <= 255
    pgmHeader = 'P5' + '\n' + str(width) + ' ' + str(height) + '\n' + str(255) +  '\n'
    pgmHeader_byte = bytearray(pgmHeader,'utf-8')
    new_map.write(pgmHeader_byte)

    for y in range(height):
        for y in range(width):
          val = map.read(1)
          if ord(val) < 254:
            new_val = b'\x00'
            new_map.write(bytes(new_val))
          else:
            new_map.write(val)
    new_map.close()
    map.close()

read_map('../maps/bremen_kitchen')
