class IndexDBDataStore {
    constructor(dbName = 'measurementDB', storeName = 'timeseriesData') {
        this.dbName = dbName;
        this.storeName = storeName;
        this.db = null;
    }

    // Initialize the database
    async init() {
        return new Promise((resolve, reject) => {
            const request = indexedDB.open(this.dbName, 1);

            request.onerror = () => reject(request.error);
            request.onsuccess = () => {
                this.db = request.result;
                resolve();
            };

            request.onupgradeneeded = (event) => {
                const db = event.target.result;
                if (!db.objectStoreNames.contains(this.storeName)) {
                    db.createObjectStore(this.storeName, { keyPath: 'timestamp' });
                }
            };
        });
    }

    async storeData(lineProtocol) {
        try {
            const [header, fields] = lineProtocol.split(' ');
            const [measurement, tags] = header.split(',');
            const [tagKey, tagValue] = tags.split('=');
            
            const fieldValues = Object.fromEntries(
                fields.split(',').map(field => {
                    const [key, value] = field.split('=');
                    return [key, isNaN(value) ? value : Number(value)];
                })
            );

            const data = {
                measurement,
                [tagKey]: tagValue,
                ...fieldValues,
                timestamp: Date.now()
            };

            await this._writeToStore(data);
            return true;
        } catch (error) {
            console.error('Error storing data:', error);
            return false;
        }
    }

    async _writeToStore(data) {
        return new Promise((resolve, reject) => {
            const transaction = this.db.transaction([this.storeName], 'readwrite');
            const store = transaction.objectStore(this.storeName);
            const request = store.put(data);
            
            request.onerror = () => reject(request.error);
            request.onsuccess = () => resolve();
        });
    }
}

// Usage example:
/*
const dataStore = new IndexDBDataStore();
await dataStore.init();
await dataStore.storeData('greenhouse,sensor_id=ESP32_001 temperature=23.5,humidity=45,moisture=60');
*/