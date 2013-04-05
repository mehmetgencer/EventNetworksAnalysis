package ena;
import org.apache.commons.cli.*;
public class enatest {
	public static void main(String[] args) throws Exception{
		Debugger.level=3;
		Options options = new Options();//See http://commons.apache.org/proper/commons-cli/javadocs/api-release/index.html
		options.addOption("v", true, "verbosity level");
		options.addOption("f", true, "number of frames in animation");
		HelpFormatter formatter = new HelpFormatter();
		formatter.printHelp( "ant", options );
		CommandLineParser parser = new BasicParser();
		CommandLine cmd = parser.parse( options, args, true);
	    Debugger.level=Integer.parseInt(cmd.getOptionValue("v", "1"));
	    int numFrames=Integer.parseInt(cmd.getOptionValue("f", "500"));
	    String fname=cmd.getArgs()[0];
	    EventNetwork en=new EventNetwork(fname);
		//RandomStaticLayout l=new RandomStaticLayout(en,2);
		//SpringLayout l=new SpringLayout(en,2);
	    //SpringLayoutWithWallRepulsion l=new SpringLayoutWithWallRepulsion(en,2);
	    SpringLayoutWithClumpProtection l=new SpringLayoutWithClumpProtection(en,2);
		l.initialize();
		/*Thread t=new Thread(l);
		t.setDaemon(true);
		t.start();*/
		new NetworkDisplayWindow(l,numFrames);
	}
}