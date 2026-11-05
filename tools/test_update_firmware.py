import importlib.util,unittest,json
from pathlib import Path
root=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('uploader',root/'update_firmware.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
class Fake:
 def __init__(self,bad_offset=False,lost_begin=False):self.calls=[];self.offset=0;self.hash='';self.size=0;self.bad=bad_offset;self.lost=lost_begin
 def call(self,c,**kw):
  self.calls.append(c);t=c['T']
  if t==625:return dict(protocol='studio-serial-ota-v1',running=bool(self.size),ready=False,paused=False,partition='app0',size=self.size,sha256=self.hash,token=123)
  if t==620:
   self.hash=c['sha256'];self.size=c['size']
   if self.lost:raise TimeoutError()
   return dict(token=123)
  if t==621:
   assert c['offset']==self.offset;self.offset+=len(bytes.fromhex(c['hex']))
   return dict(offset=self.offset+(1 if self.bad else 0))
  if t==622:
   assert self.offset==self.size
   return dict(ready=True,sha256=self.hash)
  return dict(ok=True)
class Tests(unittest.TestCase):
 def test_stale_chunk_ack_is_ignored(self):
  class Port:
   def __init__(self):self.rx=b''
   @property
   def in_waiting(self):return len(self.rx)
   def write(self,packet):
    self.packet=packet
    self.rx=b''.join((json.dumps(dict(T=6211,ok=True,token=4294967295,offset=o))+'\n').encode() for o in (1310656,1310720))
   def read(self,n):r=self.rx[:n];self.rx=self.rx[n:];return r
  port=Port();reply=mod.Protocol(port,lambda *args:None).call(dict(T=621,token=4294967295,offset=1310656,hex='ab'*64))
  self.assertEqual(reply['offset'],1310720);self.assertLessEqual(len(port.packet),256)
 def test_complete_then_reboot(self):
  f=Fake();data=b'\xe9'+b'a'*149;r=mod.upload(f,data)
  self.assertTrue(r['committed']);self.assertEqual([c['T'] for c in f.calls],[625,620,621,621,621,622,624])
 def test_offset_mismatch_never_commits(self):
  f=Fake(bad_offset=True)
  with self.assertRaises(RuntimeError):mod.upload(f,b'\xe9'+b'a'*80)
  self.assertNotIn(622,[c['T'] for c in f.calls]);self.assertEqual(f.calls[-1]['T'],623)
 def test_lost_begin_ack_recovered_by_hash(self):
  f=Fake(lost_begin=True);self.assertTrue(mod.upload(f,b'\xe9'+b'a'*80,False)['committed'])
  self.assertNotIn(624,[c['T'] for c in f.calls])
 def test_invalid_image_no_io(self):
  f=Fake()
  with self.assertRaises(ValueError):mod.upload(f,b'bad')
  self.assertEqual(f.calls,[])

if __name__=='__main__':unittest.main()
