package pack;

/**
 * @author Paolo
**********************

Avvia l'applicazione creando un SerialPortController ed un Pusher rispettivamente per:

* Gestire le comunicazioni sulla porta seriale
* Fare il pushing dei dati sul server UDP remoto
*/

public class ToushekCoordinator {
    
    public static void main (String [] args){

        new SerialPortController();
        System.out.println("Touschek Application Started...");
    }
}