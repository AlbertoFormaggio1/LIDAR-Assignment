#include "LaserScannerDriver.h"
#include <iomanip>
#include <cmath>
#include <stdexcept>
#include <algorithm>

using namespace std;

LaserScannerDriver::LaserScannerDriver(double resolution) : angular_resolution_{ resolution }, front_{ 0 }, back_{ 0 }
{
	//Impedisco di inserire risoluzioni angolari non valide: una modifica al valore inserito senza informare l'utente potrebbe dar luogo a comportamenti
	//non voluti del programma non comprensibili all'utente.
	//Ritornare un valore non è possibile perchè siamo nel costruttore, se non venisse lanciata l'eccezione l'oggetto si troverebbe in uno stato non valido
	if (isnan(resolution) || resolution < 0.1 || resolution > 1)
		throw out_of_range("Scanner resolution " + to_string(resolution) + " invalid: must be in the range [ 0.1 , 1 ]");

	//Ho allocato nel free store uno spazio per un numero di puntatori a double di dimensione pari a BUFFER_DIM 
	//E' stato allocato qui e non nella initialization list per evitare memory leaks: nel caso in cui venisse lanciata l'eccezione relativa alla risoluzione
	//il puntatore inizializzato prima di chiamare il costruttore non verrebbe deallocato automaticamente.
	buffer_ = new double* [BUFFER_DIM];	
	
	//Inizializzo il buffer settando tutti i puntatori a nullptr per rispettare le invarianti
	for (int i = 0; i < BUFFER_DIM; i++)
	{
		buffer_[i] = nullptr;
	}
}

LaserScannerDriver::~LaserScannerDriver()
{
	//Chiama clear_buffer() per deallocare tutte le scansioni e poi dealloca lo spazio riservato al buffer stesso. Questo perchè deallocando solamente il buffer
	//non verrebbero deallocate anche le scansioni
	if (buffer_)			//if eseguito perchè il distruttore viene chiamato dopo aver fatto il move che ha lasciato l'oggetto in uno stato "non valido" per cui clear_buffer non funzionerebbe
	{
		clear_buffer();
	}
	delete[] buffer_;
	buffer_ = nullptr;
}

LaserScannerDriver::LaserScannerDriver(const LaserScannerDriver& lsd) : front_{ lsd.front_ }, back_{ lsd.back_ }, angular_resolution_{ lsd.angular_resolution_ }
{
	int measurements = evalute_measurement_index(kMaxAngle, angular_resolution_) + 1;
	buffer_ = copy_buffer(lsd.buffer_, measurements);
}

LaserScannerDriver::LaserScannerDriver(LaserScannerDriver&& lsd) : front_{ lsd.front_ }, back_{ lsd.back_ }, angular_resolution_{ lsd.angular_resolution_ }, buffer_ {lsd.buffer_}
{
	//Setto a nullptr per lasciare oggetto in stato non valido ed evitare che il distruttore elimini i dati spostati nell'oggetto corrente
	lsd.buffer_ = nullptr;
	lsd.angular_resolution_ = lsd.back_ = lsd.front_ = 0;
}

//Nota sugli assegnamenti di copia e move: si è deciso di adottare la politica per cui un Driver ha semplicemente il compito di *gestire* un LIDAR. Per questo motivo
//in qualsiasi punto del programma è possibile decidere di far cambiare il LIDAR che un certo Driver sta gestendo con un altro qualsiasi, 
//operazione che comporterà il cambio dunque di risoluzione, valori inseriti nel buffer e l'indice delle scansioni più e meno recenti (ovvero front e back)
LaserScannerDriver& LaserScannerDriver::operator=(const LaserScannerDriver& lsd)
{
	//Creo una copia di sicurezza del buffer in tmp per evitare problemi dovuti all'autoassegnamento
	int measurements = evalute_measurement_index(kMaxAngle, lsd.angular_resolution_) + 1;
	double** tmp = copy_buffer(lsd.buffer_, measurements);

	//Dealloco il buffer di questo oggetto e i valori a cui sta puntando
	clear_buffer();
	delete[] buffer_;
		
	//Inserisco i nuovi valori nell'oggetto corrente
	buffer_ = tmp;
	front_ = lsd.front_;
	back_ = lsd.back_;
	angular_resolution_ = lsd.angular_resolution_;

	return *this;
}

LaserScannerDriver& LaserScannerDriver::operator=(LaserScannerDriver&& lsd)
{
	//Dealloco i vecchi elementi. Non mi preoccupo del self-assignment in quanto il parametro passato è un oggetto temporaneo
	//https://stackoverflow.com/questions/9322174/move-assignment-operator-and-if-this-rhs
	clear_buffer();
	delete[] buffer_;

	//Copio i valori in questo oggetto
	buffer_ = lsd.buffer_;
	front_ = lsd.front_;
	back_ = lsd.back_;
	angular_resolution_ = lsd.angular_resolution_;

	//Invalido l'oggetto passato
	lsd.buffer_ = nullptr;
	lsd.angular_resolution_ = lsd.back_ = lsd.front_ = 0;

	return *this;
}

//Funzione creata per evitare duplicazione di codice nei copy constructor e nel copy assignment. E' necessario passare anche values_per_scan
//perchè nel caso di copy assignment il numero di misurazioni si ottiene a partire dalla risoluzione angolare del parametro passato e non di
//questo oggetto
double** LaserScannerDriver::copy_buffer(double** from,int values_per_scan) const
{
	double** res = new double* [BUFFER_DIM];

	for (int i = 0; i < BUFFER_DIM; i++)
	{
		if (!from[i])
			res[i] = nullptr;
		else
		{
			res[i] = new double[values_per_scan];
			copy(from[i], from[i] + values_per_scan, res[i]);
		}
	}
	
	return res;
}


void LaserScannerDriver::new_scan(const vector<double>& vec)
{
	if (is_full())	//se il buffer è pieno devo deallocare la memoria occupata da scansioni precedenti, settare a null la cella(rispetto l'invariante) e spostare l'indice di front
	{
		delete[] buffer_[front_];
		buffer_[front_] = nullptr;
		front_ = next_circular_index(front_);
	}
		
	int measurements = evalute_measurement_index(kMaxAngle, angular_resolution_) + 1;	//Il numero di misurazioni totali è l'indice della misurazione in corrispondenza a maxAngle + 1
	buffer_[back_] = new double[measurements];											//Sovrascrivo il puntatore alla vecchia zona di memoria deallocata con un puntatore alla nuova

	int min_size = min(static_cast<int>(vec.size()), measurements);

	for (int i = 0; i < min_size; i++)
	{
		double element = vec[i];														//Accedo solo una volta al free store per ogni dato così da ridurre il numero totale di accessi
		//Se viene passato un NaN o un numero negativo ritengo opportuno avvisare l'utente: il LIDAR ha dei problemi nell'effettuare delle misurazioni,
		//se agissi in modo "silenzioso" (inserendo 0 ad esempio) l'utente non verrebbe a conoscenza dei problemi (gravi) del dispostivo
		if (isnan(element))												
			throw invalid_argument("Check your LIDAR! You are passing a value which is Not A Number (NaN)");
		if(element < 0)
			throw invalid_argument("Check your LIDAR! You are passing a negative distance");
		buffer_[back_][i] = element;
	}

	//Se la dimensione del vector è minore del numero di misurazioni massime, inserisco 0 nelle celle rimanenti
	for (int i = vec.size(); i < measurements; i++)
		buffer_[back_][i] = 0;

	back_ = next_circular_index(back_);
	
}


vector<double> LaserScannerDriver::get_scan()
{
	if (is_empty())
		throw EmptyBufferException();

	//Poichè posso conoscere in anticipo la dimensione del vector da restituire, alloco un vector riservando il numero corretto di elementi così da evitare ridimensionamenti in fase di inserimento
	//che peggiorano le prestazioni del metodo (in questo modo 1 inserimento: O(1) in media ===> O(1) sempre)
	int measurements = evalute_measurement_index(kMaxAngle, angular_resolution_) + 1;
	vector<double> v;
	v.reserve(measurements);

	for (int i = 0; i < measurements; i++)
	{
		v.push_back(buffer_[front_][i]);
	}

	//elimino la scansione appena rimossa
	delete[] buffer_[front_];
	buffer_[front_] = nullptr;

	//front punta alla scansione meno recente dopo quella rimossa
	front_ = next_circular_index(front_);

	//Viene ritornato un vector per valore perchè verrà usato l'assegnamento/costruttore di move della classe vector (o l'ottimizzazione da parte del compilatore di copy elision).
	return v;
}

void LaserScannerDriver::clear_buffer()
{
	//Nota sul ciclo while: elimino solo le celle che hanno un riferimento. Tutte le celle fuori dall'area del buffer valida hanno già il puntatore settato a nullptr, andare a deallocarle
	//e settare il puntatore della cella a nullptr fa diventare questa operazione O(n) => Theta(n) senza alcun motivo

	while (!is_empty())							
	{											
		delete[] buffer_[front_];
		buffer_[front_] = nullptr;
		front_ = next_circular_index(front_);
	}
	front_ = back_ = 0;
}

double LaserScannerDriver::get_distance(double angle) const
{
	if (isnan(angle))
		throw invalid_argument("The given angle is Not A Number (NaN)");

	if (is_empty())
		throw EmptyBufferException();

	int measurement_index = evalute_measurement_index(angle, angular_resolution_);	//Prende l'indice della misurazione richiesta (con eventuale arrotondamento)
	int last_scan_index = previous_circular_index(back_);							//Calcola l'indice della scansione più recente
	double distance = buffer_[last_scan_index][measurement_index];

	return distance;
}


double LaserScannerDriver::angular_resolution() const
{
	return angular_resolution_;
}


std::ostream& operator<< (std::ostream& os, const LaserScannerDriver& lsd)
{
	if (lsd.is_empty())
		os << "No scan found in the buffer. Cannot print most recent scan.";
	else
	{
		int measurements = evalute_measurement_index(LaserScannerDriver::kMaxAngle, lsd.angular_resolution()) + 1;
		constexpr int values_per_row = 4;														//Mi dice quante coppie stampare per riga
		for (int i = 0; i < values_per_row; i++)												//Stampo le intestazioni delle colonne
			os << setw(10) << "Angle" << setw(9) << " Value";
		os << endl;
		os << fixed;
		os << setprecision(3);
		for (int i = 0; i < measurements; i++)													//Stampo tutte le coppie angolo/valore (4 per riga)
		{
			double angle = i * lsd.angular_resolution();
			os << setw(9) << angle << ":" << setw(8) << lsd.get_distance(angle) << ",";

			if ((i + 1) % values_per_row == 0)
				os << endl;
		}
	}
	os << endl;
	return os;
}

//Nota sulla funzione: è stata creata come helper function perchè può essere utilizzata anche in modo scollegato dal LIDAR,
//Si può anche utilizzare per ottenere l'indice della scansione all'interno del vector. 
//
//Poichè ritorna l'indice dell'ultima scansione, per ottenere il numero di scansioni bisogna incrementare di 1 il valore ritornato da questa funzione
int evalute_measurement_index(double angle, double resolution)
{
	//Nota sul funzionamento: 
	// - la risoluzione < 0 viene controllata nel caso in cui la funzione sia invocata dall'esterno (la classe passerà sempre un parametro valido per le invarianti)
	// - Se l'angolo è nel range giusto calcolo la misurazione e arrotondo al valore più vicino con round() (come da richiesta)
	// - Se l'angolo è >= di kMaxAngle viene ritornato l'indice dell'ultima scansione eseguita (come da richiesta dell'esercizio). Viene usata la funzione floor() perchè
	//   ad esempio con resolution = 0.76 e angle = 180 -> 180/0.87 = 206.90, ma l'ultima scansione possibile è quella all'indice 206 (se arrotondassi otterrei 207, che non va bene)
	// - Se l'angolo è <= 0 viene ritornato 0 che è l'indice della scansione più vicina al grado fornito (ovvero quella di grado 0). Non viene fatto alcun controllo perchè 0 è il valore 
	//   che assegno inizialmente alla variabile da ritornare

	int measurements = 0;

	if (isnan(angle) || isnan(resolution) || resolution <= 0)
		measurements = -1;
	else if (isgreaterequal(angle, LaserScannerDriver::kMaxAngle))
		measurements = static_cast<int>(floor(LaserScannerDriver::kMaxAngle / resolution));
	else if (angle > 0)
		measurements = static_cast<int>(round(angle / resolution));

	return measurements;
}
