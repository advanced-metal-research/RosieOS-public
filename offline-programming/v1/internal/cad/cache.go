package cad

import (
	"container/list"
	"sync"
)

type cacheKey struct {
	fingerprint string
	variant     [32]byte
}

type cacheEntry struct {
	key   cacheKey
	value []byte
}

type responseCache struct {
	mu         sync.Mutex
	maxEntries int
	maxBytes   int64
	usedBytes  int64
	items      map[cacheKey]*list.Element
	lru        *list.List
}

func newResponseCache(maxEntries int, maxBytes int64) *responseCache {
	return &responseCache{
		maxEntries: maxEntries,
		maxBytes:   maxBytes,
		items:      make(map[cacheKey]*list.Element),
		lru:        list.New(),
	}
}

func (cache *responseCache) get(key cacheKey) ([]byte, bool) {
	if cache.maxEntries <= 0 || cache.maxBytes <= 0 {
		return nil, false
	}
	cache.mu.Lock()
	defer cache.mu.Unlock()
	element, ok := cache.items[key]
	if !ok {
		return nil, false
	}
	cache.lru.MoveToFront(element)
	entry := element.Value.(*cacheEntry)
	return append([]byte(nil), entry.value...), true
}

func (cache *responseCache) put(key cacheKey, value []byte) {
	size := int64(len(value))
	if cache.maxEntries <= 0 || cache.maxBytes <= 0 || size > cache.maxBytes {
		return
	}
	cache.mu.Lock()
	defer cache.mu.Unlock()
	if existing, ok := cache.items[key]; ok {
		entry := existing.Value.(*cacheEntry)
		cache.usedBytes -= int64(len(entry.value))
		entry.value = append(entry.value[:0], value...)
		cache.usedBytes += int64(len(entry.value))
		cache.lru.MoveToFront(existing)
	} else {
		entry := &cacheEntry{key: key, value: append([]byte(nil), value...)}
		cache.items[key] = cache.lru.PushFront(entry)
		cache.usedBytes += int64(len(entry.value))
	}
	for cache.lru.Len() > cache.maxEntries || cache.usedBytes > cache.maxBytes {
		cache.removeOldest()
	}
}

func (cache *responseCache) purgeFingerprint(fingerprint string) {
	cache.mu.Lock()
	defer cache.mu.Unlock()
	for key, element := range cache.items {
		if key.fingerprint != fingerprint {
			continue
		}
		cache.removeElement(element)
	}
}

func (cache *responseCache) stats() CacheStats {
	cache.mu.Lock()
	defer cache.mu.Unlock()
	return CacheStats{Entries: cache.lru.Len(), Bytes: cache.usedBytes}
}

func (cache *responseCache) removeOldest() {
	if oldest := cache.lru.Back(); oldest != nil {
		cache.removeElement(oldest)
	}
}

func (cache *responseCache) removeElement(element *list.Element) {
	entry := element.Value.(*cacheEntry)
	delete(cache.items, entry.key)
	cache.usedBytes -= int64(len(entry.value))
	cache.lru.Remove(element)
}
