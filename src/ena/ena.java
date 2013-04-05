package ena;

import java.io.*;
import java.util.*;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
//import org.json.simple.parser.ParseException;

class EventNetwork {
	String name="Unnamed network";
	Map<Integer, Node> nodes=new HashMap<Integer,Node>();
	Collection<Event> events=new TreeSet<Event>(new EventComparator());
	double[] timespan=new double[] {Double.MIN_VALUE,Double.MIN_VALUE};
	public EventNetwork(String fname) {
		try {
			BufferedReader in = new BufferedReader(new FileReader(fname));
			JSONParser parser = new JSONParser();
			int sec=0;
			while(true){
				String line = in.readLine();
				if (line==null) 
					break;
				//System.out.println(line);
				StringTokenizer tokenizer=new StringTokenizer(line,">");
				String lineType=tokenizer.nextToken();
				String lineContent=line.substring(lineType.length()+1);
				//System.out.println("Parsing:"+"{"+lineContent+"}");
				JSONObject jsonObject = (JSONObject) (parser.parse("{"+lineContent+"}")); 
				if (lineType.equals("network")) {
					String name=(String)(jsonObject.get("name"));
					this.name=name;
				}else if(lineType.equals("node")) {
					int id=((Long)(jsonObject.get("id"))).intValue();
					String name=(String)(jsonObject.get("name"));
					Node n=new Node(id,name);
					nodes.put(id, n);
				}else { //edge
					int src=((Long)(jsonObject.get("src"))).intValue();
					int dst=((Long)(jsonObject.get("dst"))).intValue();
					double time=((Long)(jsonObject.get("time"))).doubleValue();
					if (time>=timespan[1]) {
						if (timespan[0]==Double.MIN_VALUE)
							timespan[0]=time;
						timespan[1]=time;
					}
					if (src==dst) {
						Debugger.progress("SELF EDGE from %d to %d at time %g is ignored",src, dst, time );
						sec++;
					} else {
						Event e=new Event(src,dst,time);
						events.add(e);
						ensureNode(src).addEvent(e);
						ensureNode(dst).addEvent(e);
					}
				}
			}
			in.close();
			System.console().printf("Network read: %d nodes, %d edges (%d self edges are ignored)\ntimespan: %f - %f\n",nodes.size(),events.size(),sec,timespan[0],timespan[1]);
		} catch (Exception e) {
			System.out.println(e);
			e.printStackTrace();
		}
	}
	
	Node ensureNode(int id) {
		if (!nodes.containsKey(id))
			return nodes.put(id, new Node (id,""));
		else
			return nodes.get(id);
	}
}

class Node {
	int id;
	String name;
	List<Event> events=new ArrayList<Event>();
	public Node(int id, String name) {
		this.id=id;
		this.name=name;
	}
	void addEvent(Event e) {events.add(e);}
}

class Event {
	int src,dst;
	double time;
	boolean directed=true;
	double weight=1.0;
	public Event(int src, int dst, double time) {
		this.src=src;
		this.dst=dst;
		this.time=time;
	}
}

class EventComparator implements Comparator<Event> {
	public int compare(Event e1, Event e2) {
		return Double.compare(e1.time, e2.time);
	}
}

class Debugger {
	public static int level=1;
	public static boolean inProg=false;
	public static void debug(int level, String format, Object... args) {
		if (level<=Debugger.level) {
			if (inProg) {
				System.console().printf("\n");
				inProg=false;
			}
			System.console().printf(format+"\n", args);
		}
	}
	public static void debug(String format, Object... args) {
		debug(1,format,args);
	}
	public static void progress(int level, String format, Object... args) {
		if (level<=Debugger.level) {
			if (!inProg) {
				inProg=true;
			}
			System.console().printf("\r"+format, args);
		}
	}
	public static void progress(String format, Object... args) {
		progress(1,format,args);
	}
	public static void isNaN(double number){
		if(number!=number)
			try {
				throw new Exception("number is NaN");
			}catch(Exception e) {
				e.printStackTrace();
				System.exit(1);
			}
	}
}