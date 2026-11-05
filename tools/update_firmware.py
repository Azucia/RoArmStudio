"""USB serial OTA client. Requires pyserial; does not reset DTR/RTS or move servos."""
import argparse,hashlib,json,time
from pathlib import Path

class Protocol:
    def __init__(self,port,log):self.port=port;self.log=log;self.buffer=b''
    def call(self,command,timeout=4,retries=0):
        packet=(json.dumps(command,separators=(',',':'))+'\n').encode()
        if len(packet)>256:raise ValueError('Packet exceeds firmware line limit')
        for attempt in range(retries+1):
            self.log('write',command);self.port.write(packet)
            until=time.monotonic()+timeout
            while time.monotonic()<until:
                self.buffer+=self.port.read(max(1,min(self.port.in_waiting,8192)))
                while b'\n' in self.buffer:
                    line,self.buffer=self.buffer.split(b'\n',1)
                    try:reply=json.loads(line[line.index(b'{'):])
                    except (ValueError,UnicodeError):continue
                    self.log('receive',reply)
                    if reply.get('T')!=command['T']*10+1:continue
                    if 'token' in command and reply.get('token')!=command['token']:continue
                    if command['T']==621 and reply.get('ok') and reply.get('offset',-1)<command['offset']+len(command['hex'])//2:continue
                    if not reply.get('ok'):raise RuntimeError(str(reply))
                    return reply
        raise TimeoutError('No acknowledgement for T'+str(command['T']))

def upload(protocol,data,reboot=True):
    if len(data)<32 or data[0]!=0xE9 or len(data)>0x140000:raise ValueError('Invalid application image/size')
    digest=hashlib.sha256(data).hexdigest()
    status=protocol.call({'T':625})
    if status.get('protocol')!='studio-serial-ota-v1':raise RuntimeError('Unsupported firmware')
    if status.get('running') or status.get('ready') or status.get('paused'):raise RuntimeError('Existing maintenance session; inspect before starting')
    token=None;committed=False
    try:
        try:start=protocol.call(dict(T=620,size=len(data),sha256=digest,confirm='UPDATE'))
        except TimeoutError:
            start=protocol.call({'T':625})
            if not(start.get('running') and start.get('size')==len(data) and start.get('sha256')==digest):raise
        token=start['token']
        if not token:raise RuntimeError('Missing session token')
        for offset in range(0,len(data),64):
            chunk=data[offset:offset+64]
            ack=protocol.call(dict(T=621,token=token,offset=offset,hex=chunk.hex()),retries=2)
            if ack.get('offset')!=offset+len(chunk):raise RuntimeError('Acknowledged offset mismatch')
            if offset%65536==0:print(f'{offset}/{len(data)} bytes',flush=True)
        ack=protocol.call(dict(T=622,token=token),timeout=10,retries=2)
        if not ack.get('ready') or ack.get('sha256')!=digest:raise RuntimeError('Commit not confirmed')
        committed=True
        if reboot:
            try:protocol.call(dict(T=624,token=token,confirm='REBOOT'))
            except TimeoutError:pass # Do not repeat an ambiguous reboot.
        return dict(sha256=digest,size=len(data),committed=committed,reboot_requested=reboot,previous_partition=status.get('partition'))
    except Exception:
        if token and not committed:
            try:protocol.call(dict(T=623,token=token))
            except Exception:pass
        raise

def main():
    import serial,serial.tools.list_ports
    ap=argparse.ArgumentParser();ap.add_argument('image',type=Path);ap.add_argument('--port');ap.add_argument('--no-reboot',action='store_true');args=ap.parse_args()
    ports=[p for p in serial.tools.list_ports.comports() if (p.vid,p.pid)==(0x10c4,0xea60) and (not args.port or p.device==args.port)]
    if len(ports)!=1:raise RuntimeError('Specify the robot USB port with --port COMxx')
    data=args.image.read_bytes()
    logpath=Path(__file__).parent/('update-'+time.strftime('%Y%m%d-%H%M%S')+'.jsonl')
    s=serial.Serial(port=None,baudrate=115200,timeout=.05,write_timeout=1);s.dtr=s.rts=False;s.port=ports[0].device
    with logpath.open('x',encoding='utf-8') as f:
        def log(event,data):f.write(json.dumps(dict(time=time.time(),event=event,data=data))+'\n');f.flush()
        try:
            s.open();result=upload(Protocol(s,log),data,not args.no_reboot);log('result',result);print(json.dumps(result))
        finally:s.close()
if __name__=='__main__':main()
