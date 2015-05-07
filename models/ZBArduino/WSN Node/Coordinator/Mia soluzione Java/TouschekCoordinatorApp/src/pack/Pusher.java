package pack;

import java.io.IOException;
import java.net.*;

/**
 * @author Paolo
**********************

Definisce un client UDP per il pushing dei dati sul server remoto
*/

public class Pusher {
    
    // Socket e datagramma UDP da usare per la spedizione
    private DatagramSocket socket=null;
    private DatagramPacket sendPack=null;
    
    // Indirizzo IPv4 del server UDP remoto
    private final String ipv4ServerAddress="10.3.0.188";
    private InetAddress serverAddr=null;
    
    // Porta di ascolto del server UDP remoto
    private final int serverPort=10000;

    // Array di byte da inviare
    private byte [] sendData;
 
    
    public Pusher() {

// Inizializzo socket e la variabile serverAddr
        try {
            socket=new DatagramSocket();
            serverAddr=InetAddress.getByName(ipv4ServerAddress);
        }
        catch(UnknownHostException uhe) {
            System.err.println("Pusher(): Unknown server address "+uhe.toString());
            System.exit(1);
        }
        catch(SocketException e) {
            System.err.println("Pusher(): "+e.getMessage());
            System.exit(1);
        }   
    }
    
    public void sendData(String str){
/*
Il parametro formale str rappresenta la stringa da inviare al server UDP remoto
1. Creo l'array di byte da inviare
2. Creo il ddatagramma UDP da inviare
3. Invio il datagramma UDP
4. Gestisco l'eventuale eccezione
*/    
        try {
            sendData=str.getBytes();
            sendPack=new DatagramPacket(sendData, sendData.length, serverAddr, serverPort);
            socket.send(sendPack);
            System.out.println("Pusher: Sent: "+str);
        }
        catch(IOException ioe){
            System.err.println("Pusher.sendData(): "+ioe.getMessage());
        }
    }
}