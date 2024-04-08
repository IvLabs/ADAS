from pyamaze import maze,agent
import numpy as np

def heuristic(cell1,cell2):
    x1,y1=cell1
    x2,y2=cell2
    return abs(x1-x2)+abs(y1-y2)
def get_minindex(x):
    f=[]
    h=[]
    for cell in x:
        f.append(cell[0])
        h.append(cell[1])
    mini=min(f)
    minindex=f.index(mini)
    f=np.array(f)
    indices = (np.where(f == mini)[0])
    print(indices)
    if len(indices)!=1:
        mini=h[indices[0]]
        for i in range(1,len(indices)):
            if h[indices[i]]<mini:
                mini=h[indices[i]]
                minindex=indices[i]
    print(minindex)
    return minindex
 

def astar(m):
    start=(m.rows,m.cols)
    gscore={}
    fscore={}
    apath={}
    for cell in m.grid:
        gscore[cell]=10000000
        fscore[cell]=10000000

    gscore[start]=0
    fscore[start]=heuristic(start,(1,1))
    openp=[(fscore[start],heuristic(start,(1,1)),start)]
    closep=[]
    minindex=0
    while  True:
        currentcell=openp[minindex][2]
        if currentcell==(1,1):
            break
        for d in 'EWSN':
            if m.maze_map[currentcell][d]==True:
                if d=='E':
                    childcell=(currentcell[0],currentcell[1]+1)
                if d=='W':
                    childcell=(currentcell[0],currentcell[1]-1)
                if d=='S':
                    childcell=(currentcell[0]+1,currentcell[1])
                if d=='N':
                    childcell =(currentcell[0]-1,currentcell[1])
                temp_gscore=gscore[currentcell]+1
                temp_fscore=temp_gscore+heuristic(childcell,(1,1))
                if (temp_fscore<fscore[childcell]):
                    gscore[childcell]=temp_gscore
                    fscore[childcell]=temp_fscore
                    openp.append((fscore[childcell],heuristic(childcell,(1,1)),childcell))
                    apath[childcell]=currentcell
        closep.append(currentcell)
        openp.pop(minindex)
        minindex=get_minindex(openp)
    return apath
def forward_path(path):
    fwdpath={}
    cell=(1,1)
    while cell!=(10,10):
        fwdpath[path[cell]]=cell
        cell=path[cell]
    return fwdpath

m=maze(10,10)
m.CreateMaze()
path=astar(m)
fwdpath=forward_path(path)
print(fwdpath)
a=agent(m,footprints=True)
m.tracePath({a:fwdpath})
m.run()


