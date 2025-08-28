"""
hexdump.py - Simple hex/ascii memory dump utility for Python

Usage:
    from hexdump import hexdump
    hexdump(data)
"""

def hexdump(data, width=16):
    """
    Prints data as a hex+ascii hexdump.
    - data: bytes-like object
    - width: number of bytes per line (default 16)
    """
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("hexdump: data must be bytes or bytearray")

    for i in range(0, len(data), width):
        chunk = data[i:i+width]
        hexstr = ' '.join(f'{b:02X}' for b in chunk)
        asciistr = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        print(f'{i:04X}  {hexstr:<{width*3}}  {asciistr}')

if __name__ == "__main__":
    # Self-test / demo
    test_data = b'This is a sample hexdump for CAN FD data!\x00\x01\x02\x7F\x80\xFF'
    print("hexdump:")
    hexdump(test_data)
