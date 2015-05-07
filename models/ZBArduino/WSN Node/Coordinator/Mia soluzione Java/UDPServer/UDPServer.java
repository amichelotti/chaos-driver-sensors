import java.io.*;
import java.net.*;

/**
 * @author Paolo
**********************

Definisce un server UDP che stampa a video le stringhe di testo ricevute
*/

public class UDPServer {
    public static void main (String [] args)  {

	// Socket e datagramma UDP da usare in ricezione
        DatagramSocket socket=null;
	DatagramPacket recPack=null;
	
	// Porta di ascolto del server UDP
	int serverPort=10000;

	// Stringa di appoggio che contiene i caratteri ricevuti	
	String sensorsData;

	// Contiene il payload del datagramma UDP ricevuto
	byte [] recData=new byte[200];
	
/*
Avvio del socket server e gestione dell'eventuale eccezione
*/
        try {
            socket=new DatagramSocket(10000);
            System.out.println("Starting server on port: "+serverPort);
        }
        catch(SocketException se) {
            System.err.println("Failed to open socket: "+se.toString());
            System.exit(1);
        }

        while(true) {
/*
Ricezione ed elaborazione infinita dei datagrammi UDP
*/
            try {
		// Creo il datagramma UDP in ricezione con i dati ricevuti
                recPack=new DatagramPacket(recData,recData.length); 
                socket.receive(recPack);
                //System.out.println("New datagram arrived...");

		// Recupero il payload del datagramma UDP e lo stampo a video
		sensorsData=new String(recData,0,recPack.getLength());
		System.out.println(sensorsData);
            }
            catch (IOException ioe) {
                System.err.println("Error: "+ioe.getMessage());
            }
        }
    }
}
