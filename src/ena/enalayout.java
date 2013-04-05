package ena;
import java.util.*;

class NodeAvatar {
	Layout layout;
	Node node;
	double[] pos;
	double firstTime;
	private double lastRevisionTime=Double.MIN_VALUE;
	private int lastDegree=0;
	Map<Integer,EdgeAvatar> edgeMap=Collections.synchronizedMap(new HashMap<Integer,EdgeAvatar>());
	public NodeAvatar(Layout layout, Node node, double[] pos) {
		this.layout=layout;
		this.node=node;
		this.firstTime=layout.currentTime;//TODO: this is not precise??
		moveTo(pos);
	}
	public int getId() {return node.id;}
	public  double[] getPos() {return pos;}
	public  void updateEdge(NodeAvatar peer,Event e) {
			if (!edgeMap.containsKey(peer.getId())) {
				EdgeAvatar edge=new EdgeAvatar(this,peer,e.directed);
				edge.update(e.time,e.weight);
				edgeMap.put(peer.getId(), edge);
			} else 
				edgeMap.get(peer.getId()).update(e.time,e.weight);
	}
	private int reviseEdgeMap(double time) {
		if (time==lastRevisionTime)
			return lastDegree;
		synchronized (edgeMap) {
			for(int peerId:edgeMap.keySet())
				if (edgeMap.get(peerId).getWeightAtTime(time)==0)
					edgeMap.remove(peerId);
		}
			lastDegree=edgeMap.size();
		return lastDegree;
	}
	public double getDegree(double time) {
		return edgeMap.size();
		//return reviseEdgeMap(time);
	}
	public boolean isConnected(double time) {
		return getDegree(time)>0;
	}
	public  double getHeat(double time) {
		return layout.Hprime*layout.timeSinceStart(time)/((layout.timeSinceStart(time)+layout.H*(time-firstTime))*(getDegree(time)+1));
	}
	public  void moveTo(double[] newPos) {
		if (pos==null)
			layout.octree.addNode(getId(),newPos);
		else
			layout.octree.moveNode(getId(), pos, newPos);
		if (this.pos==null)
			pos=new double[newPos.length];
		for(int i=0;i<pos.length;i++)
			pos[i]=newPos[i];
		Debugger.debug(3,"Moving node %d to : %s",node.id,VectorOperations.toString(newPos));
	}
}

class EdgeAvatar  {
	Layout layout;
	NodeAvatar n1,n2;
	boolean directed;
	double weight=0,lastWeight=0;
	//List<Event> events=new ArrayList<Event>();
	double lastUpdateTime=0;
	public EdgeAvatar(NodeAvatar n1, NodeAvatar n2, boolean directed) {
		this.layout=n1.layout;
		this.n1 = n1;
		this.n2 = n2;
		this.directed=directed;
	}
	public double update(double time, double weight) {
		this.weight=layout.decay(this.weight,time-lastUpdateTime)+weight;
		lastUpdateTime=time;
		return weight;
	}
	public double getLastWeight(){return lastWeight;}
	public double getWeightAtTime(double time) {
		lastWeight=layout.decay(weight, time-lastUpdateTime);
		return lastWeight;
	}
	public double[] getPosStart() {return n1.getPos();}
	public double[] getPosEnd() {return n2.getPos();}
}

abstract class Layout implements Runnable{
	EventNetwork network;
	Map<Integer,NodeAvatar> nodeAvatars;
	int dim; //dimensions, 2 or 3
	double[][] box; //min,max points in different dimensions, thus double[dim][2] matrix
	double aspect, boxX, boxY;
	double currentTime;
	final double D; //decay rate
	final double Dprime=0.5;
	final double H=1.0;
	final double Hprime;
	final double C=0.5;
	final double theta=1.2;
	final int numRounds=1; //number of correction rounds in each advancement (event addition)
	int rounds=numRounds;
	boolean stop=false;
	int numActiveVertices=1;
	double timeActiveVerticesCounted=Double.MIN_VALUE;
	double lastIdealNodeDistance, timeLastIdealNodeDistanceComputed;
	Octree octree;
	public Layout(EventNetwork network, int dim) throws Exception {
		this(network,dim, new double[][] {{0,1},{0,1}});
	}

	public Layout(EventNetwork network, int dim, double[][] box) throws Exception {
		if (dim>3||dim<2)
			throw new Exception("Dimensions other than 2 or 3 are not supported");
		this.network=network;
		this.dim=dim;
		this.box=box;
		this.D=10/(network.timespan[1]-network.timespan[0]);
		System.console().printf("Decay: %g\n",D);
		boxX=box[0][1]-box[0][0];
		boxY=box[1][1]-box[1][0];
		aspect=boxX/boxY;
		Hprime=getBoxSide()/10.0;System.console().printf("Hprime: %g (box side:%g)\n",Hprime,getBoxSide());
		octree=new Octree(box,dim,1);
		Debugger.debug(2,"Layout box: %s",VectorOperations.toString(box));
	}

	public void initialize() {
		Debugger.debug(2,"Initializing");
		this.currentTime=network.timespan[0];
		nodeAvatars=Collections.synchronizedMap(new HashMap<Integer,NodeAvatar>());
		rounds=numRounds;
		stop=false;
	}

	public void stop() {stop=true;}
	public double timeSinceStart(double time) {
		return time-network.timespan[0];
	}
	public int getActiveVertexCount() {
		if (currentTime>timeActiveVerticesCounted) {
			int c=0;
			for(NodeAvatar n:nodeAvatars.values())
				if (n.isConnected(currentTime))
					c++;
			numActiveVertices=c;
			timeActiveVerticesCounted=currentTime;
		}
		return numActiveVertices;
	}
	public double getBoxAreaOrVolume() {
		double s=1;
		for(int i=0;i<dim;i++)
			s*=(box[i][1]-box[i][0]);
		return s;
	}
	public double getBoxSide() {
		if (dim==2)
			return Math.sqrt(getBoxAreaOrVolume());
		else 
			return Math.cbrt(getBoxAreaOrVolume());
	}
	public double idealNodeDistance() {
		if (currentTime>timeLastIdealNodeDistanceComputed) {
			if (dim==2)
				lastIdealNodeDistance=C*Math.sqrt(getBoxAreaOrVolume()/(getActiveVertexCount()+1));
			else if(dim==3)
				lastIdealNodeDistance=C*Math.cbrt(getBoxAreaOrVolume()/(getActiveVertexCount()+1));
			timeLastIdealNodeDistanceComputed=currentTime;
		}
		return lastIdealNodeDistance;
	}
	public double aspect() {//return x/y aspect ratio of box
		return aspect;
	}
	
	public double boxX(){ //return X size of box
		return boxX;
	}
	
	public double boxY(){ //return X size of box
		return boxY;
	}
	
	public NodeAvatar getNode(Integer id) {
		return nodeAvatars.get(id);
	}
	
	public Collection<NodeAvatar> getNodes() {
		return nodeAvatars.values();
	}
	
	public Collection<EdgeAvatar> getEdges() {
		Collection<EdgeAvatar> retval=new LinkedList<EdgeAvatar>();
		for (NodeAvatar n:nodeAvatars.values()) {
			retval.addAll(n.edgeMap.values());
		}
		return retval;
	}
	
	protected NodeAvatar getNode(int id) {return nodeAvatars.get(id);}	

	abstract void initNode(int id, int firstPeerId);
	
	abstract void initNode(int id);
	
	public void advance(double toTime) {
		boolean start=false;
		for (Event e: network.events) {
			if(e.time>=currentTime)
				start=true;
			if (start)
				if(e.time<toTime) {
					if (!nodeAvatars.containsKey(e.dst)) 
						initNode(e.dst);
					if (!nodeAvatars.containsKey(e.src)) 
						initNode(e.src,e.dst);
					getNode(e.src).updateEdge(getNode(e.dst),e);
					getNode(e.dst).updateEdge(getNode(e.src),e);
				} else 
					break;
		}
		currentTime=toTime;
		rounds=numRounds;
	}
	
	public void run() {
		/*for(;!stop;) {
		}*/
	}
	
	public double decay(double initialWeight, double timePassed) {
		if (timePassed==0)
			return initialWeight;
		double tmp=initialWeight*Math.exp(-D*timePassed);
		Debugger.isNaN(tmp);
		if (tmp>Dprime) {
			Debugger.debug(4,"Edge strength:%g",tmp);
			return tmp;
		}
		else {
			if (timePassed>0)
				Debugger.debug(4,"Edge is timed out");
			return 0.0f;
		}
	}
	public void work() {}
	public void adjustNode(NodeAvatar na, double time) {}
}

class RandomStaticLayout extends Layout {
	public RandomStaticLayout(EventNetwork network, int dim) throws Exception{
		super(network,dim);
	}
	public RandomStaticLayout(EventNetwork network, int dim, double[][] box) throws Exception{
		super(network,dim, new double[][] {{0,1},{0,1},{0,1}});
	}

	void initNode(int id, int firstPeerId) {
		//TODO: place near its peer
		double[] peerpos=getNode(firstPeerId).pos;
		double[] toWall=VectorOperations.vectorToNearestWall(peerpos, box);
		double[] finalPos=VectorOperations.vectorAdd(peerpos, toWall);
		/*double diag=VectorOperations.vectorLength(VectorOperations.vectorDiff(VectorOperations.boxCorners(box)[0], VectorOperations.boxCorners(box)[1]));
		double[] toCenter=VectorOperations.vectorDiff(getNode(firstPeerId).pos,VectorOperations.boxCenter(box));
		Debugger.debug(2,"Pre-Initial positioning diag:%g, toCenter %s",diag,VectorOperations.toString(toCenter));
		double[] finalpos=VectorOperations.vectorAdd(VectorOperations.boxCenter(box),toCenter);
		do {
			toCenter=VectorOperations.vectorScale(toCenter, 1.1);
			finalpos=VectorOperations.vectorAdd(VectorOperations.boxCenter(box),toCenter);
		}while(VectorOperations.vectorLength(VectorOperations.bound(finalpos,box))!=VectorOperations.vectorLength(finalpos));
		//double[] relpos=VectorOperations.sizeLimit(toCenter,diag/2);
		/*Debugger.debug(2,"Pre-Initial positioning relpos %s",VectorOperations.toString(relpos));
		double[] abspos=VectorOperations.bound(VectorOperations.vectorAdd(VectorOperations.boxCenter(box),relpos),box);
		//initNode(id);*/
		//Debugger.debug(2,"Initial positioning  %s, with respect to existing peer %s",VectorOperations.toString(abspos),VectorOperations.toString(getNode(firstPeerId).finalpos));*/
		Debugger.debug(2,"Positionin: peer at %s, toWall: %s, finalPos: %s",VectorOperations.toString(peerpos),VectorOperations.toString(toWall),VectorOperations.toString(finalPos));
		nodeAvatars.put(id, new NodeAvatar(this,network.ensureNode(id),finalPos));
	}
	void initNode(int id) {
		Random random=new Random();
		double[] pos=new double[dim];
		for(int i=0;i<dim;i++) //TODO: find a better initial position
			pos[i] = box[i][0] + ( box[i][1] - box[i][0] ) * random.nextDouble();
		Debugger.debug(2,"First node Initial positioning %s",VectorOperations.toString(pos));
		nodeAvatars.put(id, new NodeAvatar(this,network.ensureNode(id),pos));
	}
}

class SpringLayout extends RandomStaticLayout {
	public SpringLayout(EventNetwork network, int dim) throws Exception{
		super(network,dim);
	}
	public SpringLayout(EventNetwork network, int dim, double[][] box) throws Exception{
		super(network,dim, new double[][] {{0,1},{0,1},{0,1}});
	}
	public void work() {
		while(rounds>0) {
			Debugger.progress(3,"Doing round: %d",rounds);
			try{
				synchronized (nodeAvatars) {
					for(NodeAvatar n:nodeAvatars.values()) {
						adjustNode(n,currentTime);
						//n.move(VectorOperations.vectorScale(n.getForce(currentTime),0.2));
					}
				}
			}catch(Exception e){e.printStackTrace();}
			rounds--;
		}
		Debugger.debug(3, "No rounds");
		/*try {
			Thread.sleep(100);
		}catch(Exception e){e.printStackTrace();}*/
	}
	protected double[] getNodeDiff(NodeAvatar n, NodeAvatar o) {
		double distance=0;
		double[] diff=new double[n.pos.length];
		double[] opos=new double[n.pos.length];
		System.arraycopy(o.pos, 0, opos, 0, opos.length);
		boolean changed=false;
		while (distance==0) {
			diff=VectorOperations.vectorDiff(opos,n.pos);
			distance=VectorOperations.vectorLength(diff);
			if (distance==0) {
				Random random=new Random();
				for(int i=0;i<n.pos.length;i++)
					opos[i]+=random.nextDouble()/100-1/200.0;
				opos=VectorOperations.bound(opos, box);
				changed=true;
			}
		}
		if (changed)
			o.moveTo(opos);
		return diff;
	}
	public double[] getAttractiveForceBetweenNodes(NodeAvatar na, NodeAvatar other, double time) {
		double[] force=new double[dim];
		for(int i=0;i<force.length;i++)
			force[i]=0;
		double[] diff=getNodeDiff(na,other);//VectorOperations.vectorDiff(getNode(peerId).pos,na.pos);
		double distance=VectorOperations.vectorLength(diff);
		Debugger.debug(3,"Peer position:%s, distance: %g",VectorOperations.toString(other.pos), distance);
		EdgeAvatar e=na.edgeMap.get(other.getId());
		double r=e.getWeightAtTime(time);//decay(e.weight,time-e.lastUpdateTime);
		double fa, fr;
		if (r==0.0f) {
			Debugger.debug(4,"Edge %d - %d is timed out",na.getId(),other.getId());
			fa=0;
			fr=0;
		} else {
			fa=distance*distance*r/idealNodeDistance();
			fr=0;
			double f=fa+fr;
			force=VectorOperations.sizeLimit(diff, f);
			Debugger.debug(3,"Attractive Force %d from %d: %g (Edge weight %g), force vector: %s",na.getId(),other.getId(),f,r,VectorOperations.toString(force));
		}
		return force;
	}
	public double[] getRepulsiveForceOnNode(NodeAvatar na,Octree ot) {
		double[] force=new double[dim];
		for(int i=0;i<force.length;i++)
			force[i]=0;
		double d=VectorOperations.vectorLength(VectorOperations.vectorDiff(na.pos, ot.centerPos));
		Debugger.debug(3,"Octree size %g, nodeCOunt:%d, distance %g, box %s",ot.size,ot.nodeCount,d,VectorOperations.toString(ot.box));
		if ((ot.size/d)<=theta) {//faraway
			Debugger.debug(3,"Faraway");
			if (ot.nodeCount>0) { 
				double[] diff=VectorOperations.vectorDiff(na.pos,ot.centerPos);
				double distance=VectorOperations.vectorLength(diff);
				//double[] unit=VectorOperations.sizeLimit(diff,1);
				double fr= -ot.nodeCount*idealNodeDistance()*idealNodeDistance()/distance;
				force= VectorOperations.sizeLimit(diff, -fr);
			} else {
				Debugger.debug(3,"(Empty octree)");
			}
			Debugger.debug(3,"Octree force: %s", VectorOperations.toString(force));
			return force;
		} else {
			Debugger.debug(3,"Not Faraway");
			if (ot.children!=null) {
				Debugger.debug(3,"Recursing into children octrees");
				for (int i=0;i<ot.children.length;i++)
					VectorOperations.vectorAddTo(force, getRepulsiveForceOnNode(na,ot.children[i]));
			} else {
				Debugger.debug(3,"Recursing into nodes");
				synchronized(ot.nodeIds) {
					Iterator<Integer> i = ot.nodeIds.iterator(); // Must be in synchronized block
					while (i.hasNext()) {
						int nid=i.next();
						NodeAvatar a=nodeAvatars.get(nid);
						if (a.getId()==na.getId())
							continue;
						double[] diff=getNodeDiff(na,a);
						double distance=VectorOperations.vectorLength(diff);
						//double[] unit=VectorOperations.sizeLimit(diff,1);
						double fr= -idealNodeDistance()*idealNodeDistance()/distance;
						double[] peerForce=VectorOperations.sizeLimit(diff, fr);
						Debugger.debug(3,"Repulsive Force %d at %s, diff:%s : %g, force vector: %s",a.getId(),VectorOperations.toString(a.pos),VectorOperations.toString(diff),fr,VectorOperations.toString(peerForce));
						VectorOperations.vectorAddTo(force, peerForce);
					}
				}
			}
			Debugger.debug(3,"Octree force: %s", VectorOperations.toString(force));
			return force;
		}
	}
	public double[] getForceOnNode(NodeAvatar na, double time) {
		double[] force=new double[dim];
		for(int i=0;i<force.length;i++)
			force[i]=0;
		Debugger.debug(3,"\n-----------------------Adjusting node: %d, pos: %s",na.getId(), VectorOperations.toString(na.pos));
		/*if (false)
		for (int nid:nodeAvatars.keySet()) {
			NodeAvatar a=nodeAvatars.get(nid);
			if (a.getId()==na.getId())
				continue;
			double[] diff=getNodeDiff(na,a);
			double distance=VectorOperations.vectorLength(diff);
			//double[] unit=VectorOperations.sizeLimit(diff,1);
			double fr= -idealNodeDistance()*idealNodeDistance()/distance;
			double[] peerForce=VectorOperations.sizeLimit(diff, fr);
			Debugger.debug(3,"Repulsive Force %d at %s, diff:%s : %g, force vector: %s",a.getId(),VectorOperations.toString(a.pos),VectorOperations.toString(diff),fr,VectorOperations.toString(peerForce));
			VectorOperations.vectorAddTo(force, peerForce);
		} else*/
			VectorOperations.vectorAddTo(force, getRepulsiveForceOnNode(na,octree));
		Debugger.debug(3,"Repulsive Force at %s, force vector: %s",na.getId(),VectorOperations.toString(force));
		double fatmp=0;
		for (Integer peerId: na.edgeMap.keySet()) {
			/*Debugger.debug(3,"\nEdge %d -> %d, ideal distance: %g",na.getId(),peerId,idealNodeDistance());
			double[] diff=getNodeDiff(na,getNode(peerId));//VectorOperations.vectorDiff(getNode(peerId).pos,na.pos);
			double distance=VectorOperations.vectorLength(diff);
			Debugger.debug(3,"Peer position:%s, distance: %g",VectorOperations.toString(getNode(peerId).pos), distance);
			EdgeAvatar e=na.edgeMap.get(peerId);
			double r=e.getWeightAtTime(time);//decay(e.weight,time-e.lastUpdateTime);
			double fa, fr;
			if (r==0.0f) {
				Debugger.debug(2,"Edge %d - %d is timed out",na.getId(),peerId);
				fa=0;
				fr=0;
			} else {
				fa=distance*distance*r/idealNodeDistance();
				fatmp+=fa;
				fr=0;
				double f=fa+fr;
				double[] peerForce=VectorOperations.sizeLimit(diff, f);
				VectorOperations.vectorAddTo(force, peerForce);
				Debugger.debug(3,"Attractive Force %d from %d: %g (Edge weight %g), force vector: %s",na.getId(),peerId,f,r,VectorOperations.toString(peerForce));
			}*/
			double[] peerForce=this.getAttractiveForceBetweenNodes(na, getNode(peerId), time);
			double f=VectorOperations.vectorLength(peerForce);
			Debugger.debug(3,"Attractive Force %d from %d: %g , force vector: %s",na.getId(),peerId,f,VectorOperations.toString(peerForce));
			VectorOperations.vectorAddTo(force, peerForce);
			fatmp+=f;
			/*if (f==0)
				na.edgeMap.remove(peerId);*/
		}
		if (fatmp==0) {
			//Debugger.debug("NODE IS DISCONNECTED: %d",na.getId());
		}
		return force;
	}	
	public void adjustNode(NodeAvatar na, double time) {
		double[] force=getForceOnNode(na,time);
		Debugger.debug(3,"Total Force : %s",VectorOperations.toString(force));
		double nodeHeat=na.getHeat(time);
		Debugger.debug(3,"Heat of node %d: %g",na.getId(),nodeHeat);
		double[] tmp=VectorOperations.bound(
				VectorOperations.vectorAdd(na.pos, 
						VectorOperations.sizeLimit(force,nodeHeat) 
						//VectorOperations.vectorScale(force,nodeHeat)
						)
						,box);
		Debugger.debug(3,"Moving to: %s",VectorOperations.toString(tmp));
		na.moveTo(tmp);
		//System.out.println(octree);
	}
}

class SpringLayoutWithWallRepulsion extends SpringLayout {
	public SpringLayoutWithWallRepulsion(EventNetwork network, int dim) throws Exception{
		super(network,dim);
	}
	public SpringLayoutWithWallRepulsion(EventNetwork network, int dim, double[][] box) throws Exception{
		super(network,dim, new double[][] {{0,1},{0,1},{0,1}});
	}
	@Override
	public double[] getForceOnNode(NodeAvatar na, double time) {
		double[] force=super.getForceOnNode(na, time);
		double[] toWall=VectorOperations.vectorToNearestWall(na.pos, box);
		double[] wallResponse=VectorOperations.nearestWallResponse(na.pos, box);
		double distanceToWall=VectorOperations.vectorLength(toWall);
		double wallRepulsion=idealNodeDistance()*idealNodeDistance()/(Math.max(distanceToWall,this.getBoxSide()/50));
		double[] wallForce=VectorOperations.sizeLimit(wallResponse, wallRepulsion);
		Debugger.debug(5,"Wall force on %s, with wall response vector %s is %s", VectorOperations.toString(na.pos), VectorOperations.toString(wallResponse),VectorOperations.toString(wallForce));
		return VectorOperations.vectorAddTo(force, wallForce);
	}
}

class SpringLayoutWithClumpProtection extends SpringLayout {
	public SpringLayoutWithClumpProtection(EventNetwork network, int dim) throws Exception{
		super(network,dim);
	}
	public SpringLayoutWithClumpProtection(EventNetwork network, int dim, double[][] box) throws Exception{
		super(network,dim, new double[][] {{0,1},{0,1},{0,1}});
	}
	@Override
	public double[] getAttractiveForceBetweenNodes(NodeAvatar na, NodeAvatar other, double time) {
		double[] force=super.getAttractiveForceBetweenNodes(na, other, time);
		double f=VectorOperations.vectorLength(force);
		if (f==0)
			return force;
		double[] diff=getNodeDiff(na,other);
		double distance=VectorOperations.vectorLength(diff);
		double fr= (idealNodeDistance()*idealNodeDistance()/distance)*(Math.pow(na.getDegree(time)*other.getDegree(time), 2.5/2.0)-1);
		double[] peerForce=VectorOperations.sizeLimit(force, -fr);
		//Debugger.debug(3,"Repulsive Force %d at %s, diff:%s : %g, force vector: %s",a.getId(),VectorOperations.toString(a.pos),VectorOperations.toString(diff),fr,VectorOperations.toString(peerForce));
		VectorOperations.vectorAddTo(force, peerForce);
		return force;
	}
}

class Octree {
	static int depthLimit=4;
	int dim;
	int depth;
	double[][] box;
	int nodeCount;
	List<Integer> nodeIds=Collections.synchronizedList(new ArrayList<Integer>());
	double size;
	Octree[] children=null;
	double[] centerPos;
	public Octree(double[][] box,int dim, int depth){
		this.box=box;
		this.dim=dim;
		this.depth=depth;
		nodeCount=0;
		Debugger.debug(3,"Initializing some octree with box:%s",VectorOperations.toString(box));
		size=box[0][1]-box[0][0];
		centerPos=new double[box.length];
		for(int i=0;i<box.length;i++)
			centerPos[i]=(box[i][0]+box[i][1])/2;
	}
	public boolean inBox(double[] pos) {
		for (int i=0;i<pos.length;i++)
			if (pos[i]<box[i][0] || pos[i]>box[i][1])
				return false;
		return true;
	}
	private double[][][] splitAll(double[][] box){
		double[][][] boxes=split(box,0);
		for(int whichDim=1;whichDim<dim;whichDim++) {
			double[][][] tmp=new double[boxes.length*2][][];
			for(int i=0;i<boxes.length;i++) {
				double[][][] newboxes=split(boxes[i],whichDim);
				tmp[i*2]=newboxes[0];
				tmp[i*2+1]=newboxes[1];
			}
			boxes=tmp;
		}
		return boxes;
	}
	private double[][][] split(double[][] box, int whichDim) {
		double[][][] boxes=new double[2][][];
		boxes[0]=new double[box.length][box[0].length];
		boxes[1]=new double[box.length][box[0].length];
		for(int i=0;i<box.length;i++) {
			if (i==whichDim) {
				boxes[0][i][0]=box[i][0];
				boxes[0][i][1]=box[i][0]+(box[i][1]-box[i][0])/2;
				boxes[1][i][0]=box[i][0]+(box[i][1]-box[i][0])/2;
				boxes[1][i][1]=box[i][1];
			} else {
				boxes[0][i][0]=box[i][0];
				boxes[0][i][1]=box[i][1];
				boxes[1][i][0]=box[i][0];
				boxes[1][i][1]=box[i][1];
			}
		}
		Debugger.debug(3,"Taken box %s, created boxes %s and %s",VectorOperations.toString(box),VectorOperations.toString(boxes[0]),VectorOperations.toString(boxes[1]));
		return boxes;
	}
	public Octree whichChild(double[] pos) {
		for (int i=0;i<children.length;i++) {
			if (children[i].inBox(pos))
				return children[i];
		}
		return null;
	}
	public void addNode(int nodeId,double[] pos) {
		nodeCount++;
		nodeIds.add(nodeId);
		if (nodeCount>0){
			if (children==null){				 
				if (depth<depthLimit) {
					double[][][] boxes=splitAll(box);
					children=new Octree[boxes.length];
					for (int i=0;i<boxes.length;i++) {
						children[i]=new Octree(boxes[i],dim,depth+1);
					}
				} 
			} 
		}
		if (children!=null) {
			Octree tmp=whichChild(pos);
			if (tmp==null)
				Debugger.debug("Cannot position in octree while adding: pos: %s",VectorOperations.toString(pos));
			tmp.addNode(nodeId,pos);
		}
	}
	public void moveNode(int nodeId, double[] pos, double[] newPos) {
		if (children==null)
			return;
		Octree nc=whichChild(newPos);
		Octree oc=whichChild(pos);
		if (nc==null || oc==null) 
			Debugger.debug("Cannot position in octree: pos: %s, newpos: %s",VectorOperations.toString(pos),VectorOperations.toString(newPos));
		if(nc==oc)
			oc.moveNode(nodeId,pos, newPos);
		else {
			oc.removeNode(nodeId,pos);
			nc.addNode(nodeId,newPos);
		}
	}
	public void removeNode(int nodeId, double[] pos) {
		nodeCount--;
		nodeIds.remove(nodeIds.indexOf(nodeId));
		if (children!=null) {
			whichChild(pos).removeNode(nodeId,pos);
		}
	}
	public String toString() {
		String retval="";
		for(int i=0;i<depth;i++)
			retval+=" ";
		retval+=String.format("Box : %s, nodeCount:%d, depth:%d",VectorOperations.toString(box),nodeCount,depth);
		if(children!=null)
			for(int i=0;i<children.length;i++)
				retval+="\n"+children[i].toString();
		return retval;
	}
}

class VectorOperations {
	static double vectorLength(double[] vector){
		double sum=0;
		for(int i=0;i<vector.length;i++)
			sum+=vector[i]*vector[i];
		return Math.sqrt(sum);
	}
	static double[] vectorScale(double[] vector, double factor) {
		double[] scaled=new double[vector.length];
		for(int i=0;i<vector.length;i++)
			scaled[i]=vector[i]*factor;
		return scaled;
	}
	static double[] vectorDiff(double[] vector1, double[] vector2) {
		double[] diff=new double[vector1.length];
		for(int i=0;i<diff.length;i++)
			diff[i]=vector1[i]-vector2[i];
		return diff;
	}
	static double[] vectorAdd(double[] vector1, double[] vector2) {
		double[] diff=new double[vector1.length];
		for(int i=0;i<diff.length;i++)
			diff[i]=vector1[i]+vector2[i];
		return diff;
	}
	static double[] vectorAddTo(double[] vector1, double[] vector2) {
		for(int i=0;i<vector1.length;i++)
			vector1[i]+=vector2[i];
		return vector1;
	}
	static double[] bound(double[] vector, double[][] box) {
		double[] ret=new double[vector.length];
		for(int i=0;i<ret.length;i++) {
			ret[i]=vector[i];
			if (ret[i]<box[i][0])
				ret[i]=box[i][0];
			if (ret[i]>box[i][1])
				ret[i]=box[i][1];
		}
		return ret;
	}
	static double[] sizeLimit(double[] vector, double size) {
		double distance=VectorOperations.vectorLength(vector);
		if (distance==0)
			try{
				throw new Exception("Cannot size vector of length 0");
			}catch(Exception e){
				e.printStackTrace();
				System.exit(1);
			}
		//Debugger.debug(3,"Intended size: %g, real size: %g",size,VectorOperations.vectorLength(VectorOperations.vectorScale(vector,size/distance)));
		return VectorOperations.vectorScale(vector,size/distance);
	}
	static String toString(double[] vector) {
		String retval="(";
		for(int i=0;i<vector.length;i++) {
			retval+=String.format("%g",vector[i]);
			if(i<vector.length-1) retval+=",";
		}
		retval+=")";
		return retval;
	}
	static double[][] boxCorners (double[][] box) {
		double[][] retval=new double[2][];
		for (int i=0;i<2;i++) {
			retval[i]=new double[box.length];
			for(int j=0;j<box.length;j++)
				retval[i][j]=box[j][i];
		}
		return retval;
	}
	static double[] boxCenter (double[][] box) {
		double[] c1=boxCorners(box)[0];
		double[] c2=boxCorners(box)[1];
		return vectorScale(vectorAdd(c1,c2),0.5);
	}
	static double[] vectorToNearestWall(double[] pos, double[][] box) {
		int p=-1,q=-1;
		double m=Double.MAX_VALUE;
		for(int i=0;i<pos.length;i++)
			if (Math.abs(pos[i]-box[i][0])<m) {p=i;q=0;m=Math.abs(pos[i]-box[i][0]);}
			else if (Math.abs(pos[i]-box[i][1])<m) {p=i;q=1;m=Math.abs(pos[i]-box[i][1]);}
		double[] finalpos=new double[pos.length];
		for(int i=0;i<pos.length;i++)
			finalpos[i]=pos[i];
		finalpos[p]=box[p][q];
		return vectorDiff(finalpos,pos);
	}
	static double[] nearestWallResponse(double[] pos, double[][] box) {
		int p=-1,q=-1;
		double m=Double.MAX_VALUE;
		for(int i=0;i<pos.length;i++)
			if (Math.abs(pos[i]-box[i][0])<m) {p=i;q=0;m=Math.abs(pos[i]-box[i][0]);}
			else if (Math.abs(pos[i]-box[i][1])<m) {p=i;q=1;m=Math.abs(pos[i]-box[i][1]);}
		double[] finalpos=new double[pos.length];
		for(int i=0;i<pos.length;i++)
			finalpos[i]=0;
		finalpos[p]=-(box[p][q]-box[p][(q+1)%2])/Math.abs((box[p][q]-box[p][(q+1)%2]));
		return finalpos;
	}
	static String toString(double[][] box) {
		String retval="Box:";
		for(int i=0;i<box[0].length;i++) {
			double[] vector=new double[box.length];
			for(int j=0;j<vector.length;j++)
				vector[j]=box[j][i];
			retval+=toString(vector);
		}
		return retval;
	}
}