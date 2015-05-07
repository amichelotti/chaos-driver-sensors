package pack;

import gnu.io.*;
import java.io.*;
import java.util.*;

/**
 * @author Paolo
**********************

Gestisce le comunicazioni sulla porta seriale e richiede il pushing sul server UDP remoto dei dati ricevuti
*/

public class SerialPortController implements SerialPortEventListener {

    // Porta com da aprire sulla quale è connesso il modulo ZigBee
    private final String comPort="/dev/ttyUSB0";

    // Variabili di appoggio
    private Enumeration ports;
    private CommPortIdentifier comID;
    
    // Oggetto SerialPort che identifica la porta seriale aperta
    private SerialPort serialPort;
    
    // Oggetto Pusher in grado di creare il pacchetto UDP ed inviarlo al server remoto
    private final Pusher pusher;
    
    // Memorizza i dati ricevuti sulla porta seriale. Questi corrispondono alla stringa di dati formattata ed inviata dall'i-mo nodo end point della rete ZigBee (Arduino)
    private String sensorsData;
    
    // Usato per leggere dalla seriale
    private BufferedReader buffIn=null;
    
    // Usato per scrivere sulla seriale
    private PrintStream print=null;
    
    
    public SerialPortController(){      
/*
1. Richiedo l'apertura della porta com sulla quale è connesso il modulo ZigBee.
2. Creo un'istanza dell'oggetto Pusher
*/
        connect();
        pusher=new Pusher();
    }
    
    
    private void connect(){

        // Acquisisco la "lista" delle porte disponibili
        ports=CommPortIdentifier.getPortIdentifiers();
        
        // Scandisco la lista delle porte disponibili
        while(ports.hasMoreElements()){
            
            // Recupero l'identificatore della i-ma porta com
            comID = (CommPortIdentifier) ports.nextElement(); 
            System.out.println("Testing... "+comID.getName()); 

            if(comID.getPortType()==CommPortIdentifier.PORT_SERIAL && comID.getName().equals(comPort)){
/*
Si entra nell'if solo se la porta che sto testando è di tipo seriale ed è quella definita
*/
                try {                
/*
1. Provo ad aprire la porta seriale. Se entro 3 secondi la porta non è disponibile parte l'eccezione per porta in uso, altrimenti la apro ed indico il nome dell'applicazione che la sta usando (SerialPortController)
2. Aggiungo un eventListener per restare in ascolto all'infinito sulla porta
3. Chiedo di notificarmi quando sono disponibili i dati sulla seriale
4. Configuro i parametri della porta
5. Chiedo di notificarmi quando il buffer in output è vuoto dopo l'invio di uno o più byte di dati
6. Collego un buffer sull'inputStream per la lettura dei dati in arrivo sulla seriale
7. Gestisco le eventuali eccezioni
*/
                    serialPort=(SerialPort)comID.open("SerialPortController",3000);
                    serialPort.addEventListener(this);
                    serialPort.notifyOnDataAvailable(true);
                    serialPort.setSerialPortParams(9600, SerialPort.DATABITS_8,SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
                    serialPort.notifyOnOutputEmpty(true);                    
                    buffIn=new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
                    System.out.println("Connected to "+comID.getName());
                }
                catch (PortInUseException piue) {
                    System.err.println("SerialPortController: Port in use "+piue.toString());
                } 
                catch(TooManyListenersException tmle){
                    System.err.println("SerialPortController: "+tmle.toString());                        
                }
                catch(UnsupportedCommOperationException ucoe){
                    System.err.println("SerialPortController: "+ucoe.toString());
                }
                catch(IOException ioe){
                    System.err.println("SerialPortController: Opening InputStream failed "+ioe.toString());
                }
            }
        }
    }
    
    
    public void disconnect() {
// Provo a chiudere la porta com
        
        try {
            serialPort.close();
            System.out.println("Disconnected from ..." +comID.getName());
        }
        catch (Exception e) {
            System.err.println("SerialPortController: No connected..."+e.toString());
        }
    } 
 
    
    public void sendCommand(char cmd) {
/*
Invio un comando tramite la porta seriale

1. Collego un PrintStream all'outputStream della porta seriale aperta
2. Scrivo sulla seriale
3. Eseguo il flush e chiudo il PrintStrem        
*/        
        try {
            print = new PrintStream(serialPort.getOutputStream(), true);
            print.print(cmd);
            print.flush();
            print.close();
        }
        catch(IOException ioe){
            System.err.println("SerialPortController: Opening OutputStream failed "+ioe.toString());
        }
    }
    
    
    @Override
    public void serialEvent(SerialPortEvent event) {
// Il metodo viene richiamato ognivolta sono disponibili dati da leggere sulla seriale        
        
        switch (event.getEventType()) {

            case SerialPortEvent.DATA_AVAILABLE:
                try {
/*
Ogni stringa in arrivo sulla porta seriale è stata gia formattata dal nodo Arduino che l'ha generata ed inviata.
La stringa ricevuta deve quindi essere inoltrata al server remoto così come arriva. (Così ho pensato di fare ma ovviamente possiamo rivedere la cosa)
Se fossero necessarie ulteriori elaborazioni queste posso comunque essere fatte qui.
                    
Ogni pacchetto ZigBee inviato da ogni Arduino ha come payload una delle tre possibili stringhe che ho definito e che contengono i valori letti dai sensori e che sono formattate con la setssa semantica.
Queste che seguono sono 3 tipiche stringhe che possono essere ricevute:
                    

    E 1 D 1 23 56 D 2 23 59 D 3 24 50 02/04/2015 11:02
    
Come interpretarla: (Dedicata ai sensori per misure combinate di Temp ed umidità dell'aria)
                    
E 1 -> Stringa spedita dall'EndNode 1
D 1 23 56 -> Il sensore di temperatura ed umidita numero 1 ha rilevato Temperatura=23 °C ed Umidità=56 %
D 2 23 59 -> Il sensore di temperatura ed umidita numero 2 ha rilevato Temperatura=23 °C ed Umidità=59 %
D 3 23 50 -> Il sensore di temperatura ed umidita numero 3 ha rilevato Temperatura=24 °C ed Umidità=50 %
02/04/2015 11:02 -> Timestamp della misurazione con risoluzione al minuto (E' possibile aggiungere anche i secondi ma non credo ce ne sia bisogno data la natura delle grandezze fisiche monitorate)

                    
    E 3 S 1 23.58 S 2 23.05 S 3 23.69 01/04/2015 12:00
                    
Come interpretarla: (Dedicata ai sensori per misure di Temperatura superficiale delle pareti)
                    
E 3 -> Stringa spedita dall'EndNode 3
S 1 23.58 -> La sonda di temperatura superficiale numero 1 ha rilevato Temperatura=23.58 °C
S 2 23.05 -> La sonda di temperatura superficiale numero 2 ha rilevato Temperatura=23.05 °C
S 3 23.69 -> La sonda di temperatura superficiale numero 3 ha rilevato Temperatura=23.69 °C
01/04/2015 12:00 -> Timestamp della misurazione con risoluzione al minuto (E' possibile aggiungere anche i secondi ma non credo ce ne sia bisogno data la natura delle grandezze fisiche monitorate)

                    
    E 3 C 1 25567 01/04/2015 12:00
                              
Come interpretarla: (Dedicata ai sensori per misure concentrazione CO2)
                    
E 3 -> Stringa spedita dall'EndNode 3
C 1 25567 -> Il sensore per la concentrazione di CO2 numero 1 ha rilevato una concentrazione di 25567 PPM                    
01/04/2015 12:00 -> Timestamp della misurazione con risoluzione al minuto (E' possibile aggiungere anche i secondi ma non credo ce ne sia bisogno data la natura delle grandezze fisiche monitorate)                   
                    
*/                    
                      sensorsData=buffIn.readLine(); // Leggo i dati dalla seriale
                      
                    if (sensorsData.length()>5)
/*
Si entra nell'if solo se la stringa letta ha più di 5 caratteri. Evito che vengano inviate eventuali stringhe anomale.
Richiedo all'oggetto pusher di inviare al server UDP remoto la stringa appena ricevuta
Gestisco le eventuali eccezioni
*/    
                        pusher.sendData(sensorsData);

                }
                catch(IOException ioe){
                    System.err.println("SerialPortController: Opening InputStream failed "+ioe.toString());
                }
                catch(ArrayIndexOutOfBoundsException aioobe){
                    System.err.println("SerialPortController: "+aioobe.toString());
                }
            break;
        }
    }
}