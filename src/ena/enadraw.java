package ena;

import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;

class NetworkDisplayPanel extends JPanel implements ActionListener{
	private static final long serialVersionUID = 1L;
	Layout layout;
	int numFrames;
	int currentFrame=0;
	Timer timer;
	int margin=30;
	boolean sizeNodes=true;
	int maxNodeSize=40;
	int minNodeSize=3;
	public NetworkDisplayPanel(Layout layout, int numFrames) {
		super();
		this.layout=layout;
		this.numFrames=numFrames;
		timer = new Timer(200, this);
	    timer.setInitialDelay(200);
	    timer.start();
	}
	public int scaleX(double pos) {
		return margin+(int)((getWidth()-2*margin)*pos/layout.boxX());
	}
	public int scaleY(double pos) {
		return margin+(int)(((getWidth()-2*margin)/layout.aspect())*pos/layout.boxX());
	}
	public void paintComponent(Graphics g) {
		//System.out.print("\rDrawing");
		super.paintComponent(g);
    	for(NodeAvatar n: layout.getNodes()) {
           	g.setColor(Color.BLACK);
           	synchronized (n.edgeMap) {
	           	for(int eid:n.edgeMap.keySet()) {
	           		EdgeAvatar e=n.edgeMap.get(eid);
	           		if (e.getLastWeight()>0) {
	           			double[] posA=e.n1.getPos();
	           			double[] posB=e.n2.getPos();
	           			g.drawLine(scaleX(posA[0]), scaleY(posA[1]),scaleX(posB[0]), scaleY(posB[1]));
	           		}
	           	}
           	}
            g.setColor(Color.CYAN);
            double[] pos=n.getPos();
           	g.drawString(String.format("%d",n.getId()), scaleX(pos[0]), scaleY(pos[1]));
            int nodesize=3;
            if (sizeNodes)
            	nodesize=(int)Math.min(Math.sqrt(n.getDegree(layout.currentTime))+minNodeSize,maxNodeSize);
            g.setColor(Color.RED);
           	g.fillOval(scaleX(pos[0])-nodesize/2, scaleY(pos[1])-nodesize/2,nodesize,nodesize);
           	//System.out.println(String.format("\rDrawing node: %d",n.getId()));
    	}
    	/*for(EdgeAvatar e: layout.getEdges()) {//TODO: some are timed out!
            double[] posA=e.n1.getPos();
            double[] posB=e.n2.getPos();
           	g.drawLine(scaleX(posA[0]), scaleY(posA[1]),scaleX(posB[0]), scaleY(posB[1]));
    	}*/
    }
	public void actionPerformed(ActionEvent e) {
		currentFrame++;
		double t=layout.network.timespan[0]+currentFrame*(layout.network.timespan[1]-layout.network.timespan[0])/numFrames;
		Debugger.progress("Frame: %d, Advancing time to: %f", currentFrame, t);
		layout.advance(t);
		layout.work();
		if (currentFrame>=numFrames) {
			timer.stop();
			layout.stop();
			Debugger.debug("Done drawing\n");
		}
        repaint();
    }
}
class NetworkDisplayWindow extends JFrame {
	private static final long serialVersionUID = 1L;
	Layout layout;
	NetworkDisplayPanel panel;
	int sizeX=800;
	public NetworkDisplayWindow (Layout layout) {
		this(layout,500);
	}
	public NetworkDisplayWindow (Layout layout,int numFrames) {
		super("Event Network:"+layout.network.name);
		this.layout=layout;
		//layout.initialize();
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		panel=new NetworkDisplayPanel(layout,numFrames);
		add(panel);
	    setSize(sizeX, (int)(sizeX/layout.aspect()));
	    setVisible(true);
	}
}