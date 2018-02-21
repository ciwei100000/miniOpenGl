#!/usr/bin/python

# ./grading-script.py <test-dir>

import os,re,sys,shutil,random,subprocess,threading

test_dir=sys.argv[1]

# Set up scratch space for grading
dir="grading"
try:
    shutil.rmtree(dir)
except:
    pass
os.mkdir(dir)

re_cp=re.compile('\.h$|\.cpp$|^SConstruct$')
for f in os.listdir('.'):
    if re_cp.search(f):
        shutil.copyfile(f, dir+"/"+f)

# Check for cheating
token='TOKEN'+str(random.randrange(100000,999999))

header_cmd=['g++','-Wall','-g','-O3','-std=c++11','-M']
header_mini=subprocess.check_output(header_cmd+['minigl.cpp']);
header_base=subprocess.check_output(header_cmd+['header-check.cpp']);

re_header=re.compile('[\\\\ :\n]+')
ok_h={'minigl.cpp':1,'minigl.o':1,'':1}
for h in re_header.split(header_base):
    ok_h[h]=1
for h in re_header.split(header_mini):
    if not ok_h.has_key(h):
        print("FAIL: forbidden include: "+h)
        exit()

if subprocess.call('scons',cwd=dir)!=0:
    print("FAIL: Did not compile")
    exit()

def run_command_with_timeout(cmd, timeout_sec):
    proc = subprocess.Popen(cmd,cwd=dir)
    proc_thread = threading.Thread(target=proc.communicate)
    proc_thread.start()
    proc_thread.join(timeout_sec)
    if proc_thread.is_alive():
        try:
            proc.kill()
        except OSError, e:
            return True
        return False
    return True

hashed_tests={}
total_score=0

ignore_line=re.compile('^\s*(#|$)')
grade_line=re.compile('^(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s*$')
gs=0
try:
    gs=open('grading-scheme.txt')
except:
    print("FAIL: could not open grading scheme.")
    exit()

diff_parse=re.compile('diff: (.*)')
time_parse=re.compile('time: (.*)')
grade_cmd=['./minigl-nogl', '-i', 'file.txt', '-s', 'file.png', '-o', token+'.txt', '-n', '3']

for line in gs.readlines():
    if ignore_line.search(line):
        continue
    g=grade_line.search(line)
    if not g:
        print("Unrecognized command: "+line)
        exit()
    points=float(g.groups()[0])
    max_error=float(g.groups()[1])
    max_time=float(g.groups()[2])
    file=g.groups()[3]

    pass_error = 0
    pass_time = 0
    if not hashed_tests.has_key(file):
        timeout = max(int(max_time*1.2*3/1000)+1,2)
        shutil.copyfile(test_dir+'/'+file+".txt", dir+"/file.txt")
        shutil.copyfile(test_dir+'/'+file+".png", dir+"/file.png")
        if not run_command_with_timeout(grade_cmd, timeout):
            hashed_tests[file]=("TIMEOUT",None)
        else:
            results_file=open(dir+'/'+token+'.txt')
            t=time_parse.match(results_file.readline())
            d=diff_parse.match(results_file.readline())
            results_file.close()
            if t: t=float(t.groups()[0])
            if d: d=float(d.groups()[0])
            hashed_tests[file]=(d,t)

    (d,t)=hashed_tests[file]
    if d=="TIMEOUT":
        print("FAIL: (%s) Test timed out."%file)
        points=0
    elif t==None or d==None:
        print("FAIL: (%s) Program failed to report statistics."%file)
        points=0
    else:
        if d>max_error:
            print("FAIL: (%s) Too much error. Actual: %g  Max: %g."%(file,d,max_error))
            points=0
        else:
            print("PASS: (%s) diff %g vs %g."%(file,d,max_error))
        if t>max_time:
            print("FAIL: (%s) Too much time. Actual: %g  Max: %g"%(file,t,max_time))
            points=0
        else:
            print("PASS: (%s) time %g vs %g."%(file,t,max_time))

    if points>0:
        print("+%g points"%points)
        total_score+=points
    else:
        print("no points")

print("FINAL SCORE: %g"%total_score)

