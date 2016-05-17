import subprocess

def df(drives=[]):
    lines = subprocess.check_output(['df'] + drives).split('\n')
    header = map(str.strip, lines[0].replace('Mounted on', 'MountPoint').split())
    D = []
    for line in lines[1:]:
        if len(line.strip())==0:
            continue
        D.append( dict(zip(header, line.split())) )
    return D

