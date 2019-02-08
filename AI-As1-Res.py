'''
 A*, path planner solution


GROUP 14 - HAUQUE, IVANOV, TAN
'''

from tkinter import *
import struct
import xml.etree.ElementTree as ET
from queue import *
import math
import numpy as np
from numpy import long

# bounds of the window, in lat/long


LEFTLON = -78.8623
RIGHTLON = -78.832
TOPLAT = 43.8983
BOTLAT = 43.8881
WIDTH = RIGHTLON - LEFTLON
HEIGHT = TOPLAT - BOTLAT
# ratio of one degree of longitude to one degree of latitude
LONRATIO = math.cos(TOPLAT * 3.1415 / 180)
WINWID = 800
WINHGT = (int)((WINWID / LONRATIO) * HEIGHT / WIDTH)
TOXPIX = WINWID / WIDTH
TOYPIX = WINHGT / HEIGHT
# width,height of elevation array
EPIX = 3601
# approximate number of meters per degree of latitude
MPERLAT = 111000
MPERLON = MPERLAT * LONRATIO

def node_dist(n1, n2):
    ''' Distance between nodes n1 and n2, considering the elevation difference (in meters) '''
    dx = (n2.pos[0] - n1.pos[0]) * MPERLON
    dy = (n2.pos[1] - n1.pos[1]) * MPERLAT
    plain_distance = math.sqrt(dx * dx + dy * dy)
    elev_diff = n1.elev - n2.elev
    return math.sqrt(elev_diff * elev_diff + plain_distance * plain_distance)  # in meters

class Node():
    ''' Graph (map) node, not a search node! '''
    __slots__ = ('id', 'pos', 'ways', 'elev', 'waystr', 'wayset')

    def __init__(self, id, p, e=0):
        self.id = id
        self.pos = p
        self.ways = []
        self.elev = e
        self.waystr = None

    def __str__(self):
        if self.waystr is None:
            self.waystr = self.get_waystr()
        return str(self.pos) + ": " + self.waystr

    def get_waystr(self):
        if self.waystr is None:
            self.waystr = ""
            self.wayset = set()
            for w in self.ways:
                self.wayset.add(w.way.name)
            for w in self.wayset:
                self.waystr += w + " "
        return self.waystr


class Edge():
    ''' Graph (map) edge. Includes cost computation.'''
    __slots__ = ('way', 'dest', 'cost')

    def __init__(self, w, src, d):
        self.way = w
        self.dest = d
        self.cost = node_dist(src, d)
        if d.elev > src.elev:
            self.cost += (d.elev - src.elev) * 2
            if self.way.type == 'steps':
                self.cost *= 1.5


class Way():
    ''' A way is an entire street, for drawing, not searching. '''
    __slots__ = ('name', 'type', 'nodes')

    # nodes here for ease of drawing only
    def __init__(self, n, t):
        self.name = n
        self.type = t
        self.nodes = []

class Planner():
    __slots__ = ('nodes', 'ways')

    def __init__(self, n, w):
        self.nodes = n
        self.ways = w

    def heur(self, node, gnode):
        '''
        We changed the heuristic function taking into account the elevation of each node.
        '''
        return node_dist(node, gnode)

    def plan(self, s, g):
        '''
        Standard A* search
        '''
        if(s!=g):
            parents = {}
            costs = {}
            q = PriorityQueue()
            q.put((self.heur(s, g), s))
            parents[s] = None
            costs[s] = 0
            while not q.empty():
                cf, cnode = q.get()
                if cnode == g:
                    print("Path found, time will be", costs[g] * 60 / 5000, " minutes.")  # 5 km/hr known as the preferred walking distance.
                    return self.make_path(parents, g)
                for edge in cnode.ways:
                    newcost = costs[cnode] + edge.cost
                    if edge.dest not in parents or newcost < costs[edge.dest]:
                        parents[edge.dest] = (cnode, edge.way)
                        costs[edge.dest] = newcost
                        q.put((self.heur(edge.dest, g) + newcost, edge.dest))
        else:
            print('Start and goal nodes are the same! - Closing the window')
            sys.exit(0)


    def make_path(self, par, g):
        nodes = []
        ways = []
        curr = g  #Current is equal to g. Arranca de atras para adelante.
        nodes.append(curr)
        while par[curr] is not None:
            prev, way = par[curr]
            ways.append(way.name)
            nodes.append(prev)
            curr = prev
        nodes.reverse()  #Se dan vuelta los arreglos
        ways.reverse()
        return nodes, ways

class PlanWin(Frame):
    '''
    All the GUI pieces to draw the streets, allow places to be selected,
    and then draw the resulting path.
    '''

    __slots__ = ('whatis', 'nodes', 'ways', 'elevs', 'nodelab', 'elab', \
                 'planner', 'lastnode', 'startnode', 'goalnode')

    def lat_lon_to_pix(self, latlon):
        x = (latlon[1] - LEFTLON) * (TOXPIX)
        y = (TOPLAT - latlon[0]) * (TOYPIX)
        return x, y

    def pix_to_elev(self, x, y):
        return self.lat_lon_to_elev(((TOPLAT - (y / TOYPIX)), ((x / TOXPIX) + LEFTLON)))

    def lat_lon_to_elev(self, latlon):
        # row is 0 for 44N, 3601 (EPIX) for 42N
        row = (int)((44 - latlon[0]) * EPIX)
        # col is 0 for 18 E, 3601 for 19 E
        col = (int)((latlon[1] + 79) * EPIX)
        return self.elevs[row][col]

    def maphover(self, event):
        self.elab.configure(text=str(self.pix_to_elev(event.x, event.y)))
        for (dx, dy) in [(0, 0), (-1, 0), (0, -1), (1, 0), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ckpos = (event.x + dx, event.y + dy)
            if ckpos in self.whatis:
                self.lastnode = self.whatis[ckpos]
                lnpos = self.lat_lon_to_pix(self.nodes[self.lastnode].pos)
                self.canvas.coords('lastdot', (lnpos[0] - 2, lnpos[1] - 2, lnpos[0] + 2, lnpos[1] + 2))
                nstr = str(self.lastnode)
                nstr += " - "
                nstr += str(self.nodes[self.whatis[ckpos]].get_waystr())
                self.nodelab.configure(text=nstr)
                return

    def mapclick(self, event):
        ''' Canvas click handler:
        First click sets path start, second sets path goal
        '''
        print
        "Clicked on " + str(event.x) + "," + str(event.y) + " last node " + str(self.lastnode)
        if self.lastnode is None:
            return
        if self.startnode is None:
            self.startnode = self.nodes[self.lastnode]
            self.snpix = self.lat_lon_to_pix(self.startnode.pos)
            self.canvas.coords('startdot', (self.snpix[0] - 5, self.snpix[1] - 5, self.snpix[0] + 5, self.snpix[1] + 5))
        elif self.goalnode is None:
            self.goalnode = self.nodes[self.lastnode]
            self.snpix = self.lat_lon_to_pix(self.goalnode.pos)
            self.canvas.coords('goaldot', (self.snpix[0] - 5, self.snpix[1] - 5, self.snpix[0] + 5, self.snpix[1] + 5))

    def clear(self):
        ''' Clear button callback. '''
        self.lastnode = None
        self.goalnode = None
        self.startnode = None
        self.canvas.coords('startdot', (0, 0, 0, 0))
        self.canvas.coords('goaldot', (0, 0, 0, 0))
        self.canvas.coords('path', (0, 0, 0, 0))

    def plan_path(self):
        ''' Path button callback, plans and then draws path.'''
        print ("Planning Began!")
        if self.startnode is None or self.goalnode is None:
            print("Start or goal nodes haven't been defined yet")
            return
        print("From", self.startnode.id, "to", self.goalnode.id)
        nodes, ways = self.planner.plan(self.startnode, self.goalnode)
        lastway = ""
        for wayname in ways:
            if wayname != lastway:
                print (wayname)
                lastway = wayname
        coords = []
        for node in nodes:
            npos = self.lat_lon_to_pix(node.pos)
            coords.append(npos[0])
            coords.append(npos[1])
            print("Coords: \n", coords)
            # print node.id
        self.canvas.coords('path', *coords)

    def __init__(self, master, nodes, ways, coastnodes, elevs):
        self.whatis = {}
        self.nodes = nodes
        self.ways = ways
        self.elevs = elevs
        self.startnode = None
        self.goalnode = None
        self.planner = Planner(nodes, ways)
        thewin = Frame(master)
        w = Canvas(thewin, width=WINWID, height=WINHGT)  # , cursor="crosshair")
        w.bind("<Button-1>", self.mapclick)
        w.bind("<Motion>", self.maphover)
        for waynum in self.ways:
            nlist = self.ways[waynum].nodes
            thispix = self.lat_lon_to_pix(self.nodes[nlist[0]].pos)
            if len(self.nodes[nlist[0]].ways) > 2:
                self.whatis[((int)(thispix[0]), (int)(thispix[1]))] = nlist[0]
            for n in range(len(nlist) - 1):
                nextpix = self.lat_lon_to_pix(self.nodes[nlist[n + 1]].pos)
                self.whatis[((int)(nextpix[0]), (int)(nextpix[1]))] = nlist[n + 1]
                w.create_line(thispix[0], thispix[1], nextpix[0], nextpix[1])
                thispix = nextpix
        #thispix = self.lat_lon_to_pix(self.nodes[coastnodes[0]].pos)
        ## also draw the coast:
        #for n in range(len(coastnodes) - 1):
        #    nextpix = self.lat_lon_to_pix(self.nodes[coastnodes[n + 1]].pos)
        #    w.create_line(thispix[0], thispix[1], nextpix[0], nextpix[1], fill="blue")
        #    thispix = nextpix

        # other visible things are hiding for now...
        w.create_line(0, 0, 0, 0, fill='orange', width=3, tag='path')

        w.create_oval(0, 0, 0, 0, outline='green', fill='green', tag='startdot')
        w.create_oval(0, 0, 0, 0, outline='red', fill='red', tag='goaldot')
        w.create_oval(0, 0, 0, 0, outline='blue', fill='blue', tag='lastdot')
        w.pack(fill=BOTH)
        self.canvas = w

        cb = Button(thewin, text="Clear", command=self.clear)
        cb.pack(side=RIGHT, pady=5)

        sb = Button(thewin, text="Plan!", command=self.plan_path)
        sb.pack(side=RIGHT, pady=5)

        nodelablab = Label(thewin, text="Node:")
        nodelablab.pack(side=LEFT, padx=5)

        self.nodelab = Label(thewin, text="None")
        self.nodelab.pack(side=LEFT, padx=5)

        elablab = Label(thewin, text="Elev:")
        elablab.pack(side=LEFT, padx=5)

        self.elab = Label(thewin, text="0")
        self.elab.pack(side=LEFT, padx=5)

        thewin.pack()

def build_graph(elevs):
    ''' Build the search graph from the OpenStreetMap XML. '''
    tree = ET.parse('South Oshawa Map.osm')
    root = tree.getroot()

    nodes = dict()
    ways = dict()
    waytypes = set()
    coastnodes = []
    for item in root:
        if item.tag == 'node':
            coords = ((float)(item.get('lat')), (float)(item.get('lon')))
            # row is 0 for 44N
            erow = (int)((44 - coords[0]) * EPIX)
            # col is 0 for 79 W
            ecol = (int)((coords[1] + 79) * EPIX)
            try:
                el = elevs[erow][ecol]
            except IndexError:
                el = 0
            nodes[(long)(item.get('id'))] = Node((long)(item.get('id')), coords, el)
        elif item.tag == 'way':
            useme = False
            oneway = False
            myname = 'unnamed way'
            for thing in item:
                if thing.tag == 'tag' and thing.get('k') == 'highway':
                    useme = True
                    mytype = thing.get('v')
                if thing.tag == 'tag' and thing.get('k') == 'name':
                    myname = thing.get('v')
                if thing.tag == 'tag' and thing.get('k') == 'oneway':
                    if thing.get('v') == 'yes':
                        oneway = True
            if useme:
                wayid = (long)(item.get('id'))
                ways[wayid] = Way(myname, mytype)
                nlist = []
                for thing in item:
                    if thing.tag == 'nd':
                        nlist.append((long)(thing.get('ref')))
                thisn = nlist[0]
                for n in range(len(nlist) - 1):
                    nextn = nlist[n + 1]
                    nodes[thisn].ways.append(Edge(ways[wayid], nodes[thisn], nodes[nextn]))
                    thisn = nextn
                if not oneway:
                    thisn = nlist[-1]
                    for n in range(len(nlist) - 2, -1, -1):
                        nextn = nlist[n]
                        nodes[thisn].ways.append(Edge(ways[wayid], nodes[thisn], nodes[nextn]))
                        thisn = nextn
                ways[wayid].nodes = nlist
    return nodes, ways, coastnodes

def build_elevs():    #Code adapted from asuggested code in the internet - Reference: https://bit.ly/2E1rIYs
    height = EPIX
    width = EPIX
    fi = open(r"n43_w079_1arc_v2.bil", "rb")
    contents = fi.read()
    fi.close()
    s = "<%dH" % (int(width * height),)
    z = struct.unpack(s, contents)
    heights = np.zeros((height, width))
    for r in range(0, height):
        for c in range(0, width):
            elevation = z[((width) * r) + c]
            heights[r][c] = float(elevation)
    print(len(heights))
    return heights

elevs = build_elevs()
nodes, ways, coastnodes = build_graph(elevs)
print(elevs)

master = Tk()
thewin = PlanWin(master, nodes, ways, coastnodes, elevs)
mainloop()