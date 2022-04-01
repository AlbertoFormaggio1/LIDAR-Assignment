/*!
*  @author Formaggio Alberto
*  @date 3/12/2020
*/

#include <vector>
#include <string>
#include <iostream>


// Invarianti:
// - angular_resolution_ > 0.1 && angular_resolution_ < 1
// - front_ >= 0 && front_ < BUFFER_DIM
// - front_ è l'indice del puntatore alla scansione meno recente (la prima da rimuovere) o nullptr se il buffer è vuoto
// - back_ >= 0 && back_ < BUFFER_DIM
// - back_ è l'indice del puntatore alla locazione di buffer_ in cui inserire una nuova scansione. (buffer_[back] != nullptr se buffer pieno, buffer[back_] == nullptr altrimenti).
// - buffer_ è allocato con un array (di puntatori) di dimensione BUFFER_DIM
// - le celle di buffer_ non valide == nullptr
// - le costanti all'interno del codice devono avere valori validi già in fase di compilazione:
//       - kMaxAngle > 0
//       - BUFFER_DIM >= 0
//       - kDefaultResolution >= 0.1 && kDefaultResolution <= 1
class LaserScannerDriver
{
public:
	/*!
	 * @brief Angolo massimo raggiungibile da una scansione del lidar
	 * Essendo costante posso renderla anche pubblica, sarà utilizzabile in fase di solo accesso.
	 * Utile per evitare di avere "magic numbers" sparsi per il codice
	*/
	static constexpr double kMaxAngle = 180;

	/*!
	 * @brief Eccezione lanciata se si fanno operazioni non consentite su buffer vuoto
	*/
	class EmptyBufferException {};

	/*!
	 * @brief Crea una nuova istanza di LaserScannerDriver.
	 * @details Deve essere chiamato esplicitamente per evitare la conversione che in questo caso è indesiderata
	 * @throws std::out_of_range se resolution non è nel range [0.1 , 1]
	*/
	explicit LaserScannerDriver(double resolution = kDefaultResolution);
	/*!
	 * @brief Distruttore di LaserScannerDriver. Rilascia la memoria
	*/
	~LaserScannerDriver();
	/*!
	 * @brief copy constructor
	*/
	LaserScannerDriver(const LaserScannerDriver& lsd);
	/*!
	 * @brief move constructor
	 */
	LaserScannerDriver(LaserScannerDriver&& lsd);

	/*!
	 * @brief Copy assignment, esegue una deep copy.
	*/
	LaserScannerDriver& operator=(const LaserScannerDriver& lsd);
	/*!
	 * @brief Move assignment
	*/
	LaserScannerDriver& operator=(LaserScannerDriver&& lsd);

	/*!
	 * @brief Inserisce la scansione fornita nel vector all'interno del buffer. 
	*/
	void new_scan(const std::vector<double>& v);
	/*!
	 * @brief Ritorna la scansione più vecchia, eliminandola dal buffer.
	 * @throws EmptyBufferException qualora il buffer sia vuoto
	*/
	std::vector<double> get_scan();
	/*!
	 * @brief Elimina tutte le scansioni
	*/
	void clear_buffer();
	/*!
	 * @brief Ritorna la distanza della scansione più recente presente all'angolo fornito. Si approssima al valore più vicino se tale angolo
	 * non è presente
	 * @throws EmptyBufferException qualora il buffer sia vuoto
	*/
	double get_distance(double angle) const;
	/*!
	 * @brief Accessor che ritorna la risoluzione angolare di questo oggetto
	 * @details Stesso nome della variabile di esemplare per l'accessor
	*/
	double angular_resolution() const;

	//Nota di progettazione:
	//I seguenti metodi son stati resi pubblici per far sapere all'esterno se il buffer è pieno o vuoto. Usando tali metodi  l'utente può sapere se un'invocazione futura 
	//di new_scan() sovrascriverà una lettura (buffer pieno) o se una futura invocazione di get_scan() / get_distance() può lanciare eccezione (buffer vuoto) e gestire tale caso
	inline bool is_empty() const { return buffer_[front_] == nullptr; }
	inline bool is_full() const { return buffer_[back_] != nullptr; }

private:
	/*! 
	*  @brief Dimensione del buffer
	*/
	static constexpr int BUFFER_DIM = 2;	

	//NOTA DI PROGETTAZIONE:
	//Il buffer è stato allocato nel free store come da specifiche dell'esercizio. Nel buffer saranno presenti solo puntatori a elementi validi,
	//cioè una volta che una scasione sarà rimossa dal buffer, questa verrà anche deallocata (anzichè semplicemente spostare l'indice front_). 
	//Questo mi permette di tenere riservato in memoria solo lo spazio che è effettivamente necessario
	double** buffer_;

	//Default initializer
	static constexpr double kDefaultResolution = 1;

	//NOTA DI PROGETTAZIONE:
	//Si fa notare come sia stata inserita solo la risoluzione angolare e non il numero di misurazioni che possono essere eseguite per ciascuna scansione.
	//Questo perchè tale valore può essere ricavato in un tempo costante (prestazioni O(1)) grazie al metodo evaluate_measurement_index(), quindi non vi  
	//era motivo di riservare uno spazio in memoria per tale variabile. Verrà calcolata all'occorrenza.

	/*!
	 * @brief La risoluzione angolare dell'oggetto
	 * @details E' stata dichiarata privata e non costante per permettere di eseguire l'assegnamento di copia e di move (vedi spiegazione a riguardo nella **definizione** dei
	 * relativi metodi). Essendo privata e nessun metodo vi può accedere (se non in lettura) rispetta la specifica secondo cui la risoluzione angolare del LIDAR
	 * non può cambiare durante tutta la sua vita
	*/
	double angular_resolution_;

	int front_;	//Punta alla scansione meno recente
	int back_;	//Punta alla prossima locazione in cui inserire

	/*!
	 * @brief Ritorna l'indice successivo nel buffer circolare dell'indice passato (eventualmente ricominciando dalla posizione 0)
	 * @param index L'indice di cui calcolare il successivo
	 * @return L'indice succesivo di index
	*/
	inline int next_circular_index(int index) const { return (index + 1) % BUFFER_DIM; }
	/*!
	 * @brief Ritorna l'indice precedente nel buffer circolare dell'indice passato (eventualmente ricominciando dalla posizione (BUFFER_DIM - 1))
	 * @param index L'indice di cui calcolare il precedente
	 * @return L'indice precedente di index
	*/
	inline int previous_circular_index(int index) const { return (index - 1 + BUFFER_DIM) % BUFFER_DIM; }
	/*!
	 * @brief Esegue una deep copy dei valori del buffer passato in un nuovo buffer di dimensione BUFFER_DIM
	 * @param from buffer originario
	 * @param values_per_scan numero di valori da copiare
	 * @return buffer con gli elementi di from
	*/
	double** copy_buffer(double** from, int values_per_scan) const;
};

/*!
 * @brief Stampa i valori del LaserScannerDriver su os.
 * @param os stream di output di origine
 * @param lsd oggetto di origine
 * @return os con i valori di lsd formattati opportunamente
*/
std::ostream& operator<< (std::ostream& os, const LaserScannerDriver& lsd);
/*!
 * @brief Calcola l'indice della misurazione corrente data la risoluzione angolare.
 *
 * @param angle un angolo
 * @param resolution una risoluzione angolare
 * @return L'indice della misurazione. Se l'angolo è troppo grande, ritorna il numero dell'ultima misurazione eseguita (più piccola di kMaxAngle). Se l'angolo è minore di 0 ritorna 0.
 *		   Se resolution < 0 ritorna -1
*/
int evalute_measurement_index(double angle, double resolution);